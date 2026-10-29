#!/usr/bin/env python3
"""Task C: selected ArUco ID -> calibrated monocular pose -> odom PI dock.
ROS 2 Humble / Gazebo, 90-degree perspective camera; no Stadium coordinates.
Run with --marker-id N (dry-run); add --move only after checking goal.
"""
import argparse, math, time
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


def wrap(x): return math.atan2(math.sin(x), math.cos(x))
def clip(x,lo,hi): return max(lo,min(hi,x))

class Dock(Node):
    def __init__(self, marker_id, move=False, target_clearance=.10,
                 clearance_tolerance=.005, scan_topic='/scan'):
        super().__init__('aruco_task_c_pnp')
        self.marker_id=int(marker_id); self.move=bool(move)
        self.target_clearance=float(target_clearance)
        self.clearance_tolerance=float(clearance_tolerance)
        self.hard_min_clearance=.075
        ar=cv2.aruco
        self.dictionary=ar.getPredefinedDictionary(ar.DICT_5X5_50)
        self.detparams=ar.DetectorParameters() if hasattr(ar,'DetectorParameters') else ar.DetectorParameters_create()
        if hasattr(self.detparams,'cornerRefinementMethod') and hasattr(ar,'CORNER_REFINE_SUBPIX'):
            self.detparams.cornerRefinementMethod=ar.CORNER_REFINE_SUBPIX
        self.detector=ar.ArucoDetector(self.dictionary,self.detparams) if hasattr(ar,'ArucoDetector') else None
        self.bridge=CvBridge()
        self.K=None; self.D=None; self.odom=None; self.scan=None; self.pose_sample=None
        self.odom_at=0.; self.scan_at=0.; self.image_at=0.; self.frame_at=0.
        self.samples=deque(maxlen=12); self.goal=None; self.state='MEASURE'
        self.start=time.monotonic(); self.last_log=0.; self.last_motion=time.monotonic()
        self.lost_since=None; self.last_loss_log=0.0; self.dock_good_frames=0
        self.scan_topic = scan_topic
        self.sensor_wait_start = None
        self.last_sensor_wait_log = 0.0
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(Image,'/camera/image_raw',self.on_image,10)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.on_info,10)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        # LiDAR is normally best-effort: default QoS=10 asks for reliable
        # delivery and can silently match zero sensor publishers.
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_timer(.1,self.tick)
        self.get_logger().info(
            f'PNP_TASK_C READY target={marker_id} motion={move} '
            f'clearance={self.target_clearance:.3f}+/-{self.clearance_tolerance:.3f}m '
            f'scan={scan_topic} sensor_QoS=best_effort')

    def on_info(self,m):
        k=np.array(m.k,dtype=np.float64).reshape(3,3)
        if k[0,0]>20 and k[1,1]>20:
            self.K=k; self.D=np.array(m.d,dtype=np.float64).reshape(-1,1) if m.d else np.zeros((5,1))

    def on_odom(self,m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.odom=(p.x,p.y,yaw); self.odom_at=time.monotonic()

    def on_scan(self,m): self.scan=m; self.scan_at=time.monotonic()

    def on_image(self,m):
        if self.state in ('COMPLETE','ERROR'): return
        try:
            self.frame_at=time.monotonic()
            im=self.bridge.imgmsg_to_cv2(m,'bgr8')
            if self.detector is not None: cs,ids,_=self.detector.detectMarkers(im)
            else: cs,ids,_=cv2.aruco.detectMarkers(im,self.dictionary,parameters=self.detparams)
            visible=[] if ids is None else ids.flatten().tolist()
            now=time.monotonic()
            if self.marker_id not in visible:
                if now-self.last_log>2:
                    self.get_logger().info(f'VISIBLE {visible}; waiting for {self.marker_id}')
                    self.last_log=now
                return
            corners=np.asarray(cs[visible.index(self.marker_id)],dtype=np.float64).reshape(4,2)
            if self.K is None:
                if now-self.last_log>2:
                    self.get_logger().warning('Waiting for camera_info positive K')
                    self.last_log=now
                return
            side=.100; h=side/2
            object_pts=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],dtype=np.float64)
            ok,rvec,tvec=cv2.solvePnP(object_pts,corners,self.K,self.D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: return
            proj,_=cv2.projectPoints(object_pts,rvec,tvec,self.K,self.D)
            reproj=float(np.sqrt(np.mean(np.sum((proj.reshape(-1,2)-corners)**2,axis=1))))
            if reproj>2.0 or tvec[2,0] <= .10 or tvec[2,0]>2.0: return
            Rcv,_=cv2.Rodrigues(rvec)
            normal=Rcv[:,2].copy() # marker +Z in camera optical coordinates
            if normal[2]>0: normal=-normal
            # Optical frame: x right, y down, z forward. Base: x forward, y left.
            marker_base=np.array([.040+float(tvec[2,0]),-float(tvec[0,0])])
            outward_base=np.array([float(normal[2]),-float(normal[0])])
            norm=float(np.linalg.norm(outward_base))
            if norm<.5: return
            outward_base/=norm
            if self.odom is None or now-self.odom_at>.5: return
            rx,ry,yaw=self.odom; c,s=math.cos(yaw),math.sin(yaw)
            Rot=np.array([[c,-s],[s,c]])
            marker_xy=np.array([rx,ry])+Rot@marker_base
            outward=Rot@outward_base
            outward_yaw=math.atan2(outward[1],outward[0])
            # Keep robot collision front ~10 cm off wall (+3.8cm front offset).
            goal_xy=marker_xy+.138*outward
            goal_yaw=wrap(outward_yaw+math.pi)
            self.pose_sample=(float(goal_xy[0]),float(goal_xy[1]),goal_yaw,
                              float(marker_xy[0]),float(marker_xy[1]),reproj)
            self.image_at=now
            self.lost_since=None
            if self.state=='MEASURE': self.samples.append(self.pose_sample)
            if now-self.last_log>1.5:
                self.get_logger().info(f'ID={self.marker_id} PNP tvec={np.round(tvec.flatten(),3).tolist()} '
                    f'reproj={reproj:.2f}px marker_odom=({marker_xy[0]:.3f},{marker_xy[1]:.3f}) '
                    f'outward={math.degrees(outward_yaw):+.1f}deg')
                self.last_log=now
        except Exception as e:
            self.get_logger().error(f'POSE_ERROR: {e}')

    def cmd(self,v=0.,w=0.):
        m=Twist();m.linear.x=float(v);m.angular.z=float(w)
        if rclpy.ok(): self.pub.publish(m)

    def front_clearance(self):
        if self.scan is None: return None
        m=self.scan; ranges=[]
        for i,r in enumerate(m.ranges):
            angle=m.angle_min+i*m.angle_increment
            if abs(angle)<math.radians(7) and math.isfinite(r) and m.range_min<r<m.range_max:
                ranges.append(r)
        if len(ranges)<3: return None
        return float(np.percentile(ranges,20))-.070 # laser plane to robot front

    def fail(self,message):
        self.cmd();self.state='ERROR';self.get_logger().error(message)

    def tick(self):
        now=time.monotonic()
        if self.state in ('READY','COMPLETE','ERROR'):return
        if self.state=='MEASURE':
            if len(self.samples)>=8:
                A=np.array(self.samples)
                headings=A[:,2]
                heading=math.atan2(np.mean(np.sin(headings)),np.mean(np.cos(headings)))
                if max(np.ptp(A[:,0]),np.ptp(A[:,1]))<.045 and np.max(np.abs(np.arctan2(np.sin(headings-heading),np.cos(headings-heading))))<math.radians(7):
                    self.goal=(float(np.median(A[:,0])),float(np.median(A[:,1])),heading)
                    self.get_logger().info(f'MARKER_ODOM=({np.median(A[:,3]):.3f},{np.median(A[:,4]):.3f})')
                    self.get_logger().info(f'DOCK_GOAL odom=({self.goal[0]:.3f},{self.goal[1]:.3f},{math.degrees(heading):+.1f}deg)')
                    self.state='WAIT_SENSORS' if self.move else 'READY'
                    self.sensor_wait_start=now
                    return
            if now-self.start>35: self.fail('NO_POSE: check camera intrinsics, marker geometry and odom')
            return
        # Do not command docking motion before BOTH odometry and a fresh
        # laser scan have arrived. This also avoids meaningless 6000s scan_age
        # values that result from subtracting initial timestamp 0.
        if self.state == 'WAIT_SENSORS':
            odom_ok = self.odom is not None and now-self.odom_at <= .6
            scan_ok = self.scan is not None and now-self.scan_at <= .6
            if odom_ok and scan_ok and self.front_clearance() is not None:
                self.get_logger().info(
                    f'DOCK_SENSORS_READY scan={self.scan_topic}; starting approach')
                self.state = 'APPROACH'
                return
            self.cmd()  # zero motion, never bypass the LiDAR requirement
            if now-self.last_sensor_wait_log > 2.0:
                self.get_logger().warning(
                    f'WAIT_SENSORS odom={odom_ok} scan={scan_ok} '
                    f'scan_topic={self.scan_topic} '
                    f'scan_age={"never" if self.scan_at==0 else f"{now-self.scan_at:.2f}s"}')
                self.last_sensor_wait_log = now
            if now-self.sensor_wait_start > 8.0:
                self.fail(f'NO_FRESH_SCAN: no usable LaserScan on {self.scan_topic}; '
                          f'check ros2 topic info -v {self.scan_topic} and publisher QoS')
            return
        # The marker can leave the 90-degree camera near the dock. Once the
        # pose is locked, do not treat a missing image as stale odometry.
        # Continue only for a short, bounded final approach using the locked
        # odom-frame goal; never bypass odometry or laser freshness checks.
        if self.odom is None or now-self.odom_at>.6 or now-self.scan_at>.6:
            self.fail(f'SENSOR_STALE odom_age={now-self.odom_at:.2f}s '
                         f'scan_age={now-self.scan_at:.2f}s '
                         f'frame_age={now-self.frame_at:.2f}s '
                         f'marker_age={now-self.image_at:.2f}s');return
        clearance=self.front_clearance()
        if clearance is None:
            self.fail('FRONT_LIDAR_MISSING');return
        if clearance<self.hard_min_clearance:
            self.fail(f'CLEARANCE_STOP={clearance:.3f}m');return
        if now-self.start>130:
            self.fail('RUN_TIMEOUT');return
        x,y,yaw=self.odom;gx,gy,gyaw=self.goal
        dx,dy=gx-x,gy-y
        dist_to_goal=math.hypot(dx,dy)
        lon=math.cos(gyaw)*dx+math.sin(gyaw)*dy
        lat=-math.sin(gyaw)*dx+math.cos(gyaw)*dy
        hd=wrap(gyaw-yaw)

        # LiDAR is the final authority for the marking requirement.  Require
        # the robot to be near the locked visual goal and well aligned, then
        # accept 10 cm within the configured tolerance.  Check this BEFORE a
        # marker-loss timeout so a completed dock cannot be reported as ERROR.
        clearance_ok=(abs(clearance-self.target_clearance)
                      <= self.clearance_tolerance)
        pose_ok=(dist_to_goal<.080 and abs(lat)<.025
                 and abs(hd)<math.radians(4.0))
        if clearance_ok and pose_ok:
            self.cmd()
            self.dock_good_frames+=1
            if self.dock_good_frames>=3:
                self.state='COMPLETE'
                self.get_logger().info(
                    f'DOCK COMPLETE clearance={clearance:.3f}m '
                    f'dist={dist_to_goal:.3f}m lateral={lat:.3f}m '
                    f'heading={math.degrees(hd):+.1f}deg')
            return
        self.dock_good_frames=0

        image_lost=(now-self.image_at>.75)
        if now-self.frame_at > 2.5 and dist_to_goal > .16:
            self.fail('CAMERA_STREAM_STALE_OUTSIDE_FINAL_WINDOW');return
        if image_lost:
            if self.lost_since is None:
                self.lost_since=now
                self.get_logger().warning(
                    f'TARGET_LOST: using locked odom goal for short final approach; '
                    f'distance={dist_to_goal:.3f}m clearance={clearance:.3f}m')
            # Camera-free finishing is permitted ONLY near the previously
            # validated goal. Never keep travelling across the arena blindly.
            # A 90-degree camera can lose the marker during docking.
            # Permit a bounded odometry-only segment with current LiDAR
            # safety; do not allow arbitrary travel on a lost target.
            if dist_to_goal>.55 or now-self.lost_since>25.0:
                self.fail(f'TARGET_LOST_FALLBACK_LIMIT: dist={dist_to_goal:.3f}m '
                          f'elapsed={now-self.lost_since:.1f}s');return
            if now-self.last_loss_log>2:
                self.get_logger().warning(
                    f'FINAL_ODOM_ONLY dist={dist_to_goal:.3f}m '
                    f'clearance={clearance:.3f}m age={now-self.lost_since:.1f}s')
                self.last_loss_log=now
        else:
            self.lost_since=None
        if lon<-.08:
            self.fail('PASSED_GOAL');return
        desired=wrap(gyaw+math.atan2(1.8*lat,max(.14,lon)))
        err=wrap(desired-yaw)
        w=clip(.8*err,-.28,.28)

        # Fast while far away, progressively slower near the required 10 cm.
        # Near the locked pose, use LiDAR clearance error to finish the last
        # few centimetres even if PnP/odometry and wall distance disagree.
        if clearance>.40: v_cap=.070
        elif clearance>.25: v_cap=.050
        elif clearance>.16: v_cap=.030
        else: v_cap=.015
        if dist_to_goal<.20 and abs(lat)<.05 and abs(hd)<math.radians(12):
            forward_error=max(0.,clearance-self.target_clearance)
        else:
            forward_error=max(0.,lon)
        v=min(v_cap,.55*forward_error)*clip(1-1.2*abs(err),0,1)
        if abs(err)>math.radians(45): v=0.
        self.cmd(v,w)
        if now-self.last_motion>1:
            self.get_logger().info(f'APPROACH lon={lon:.3f} lat={lat:.3f} hd={math.degrees(hd):+.1f} '
                f'front_clearance={clearance:.3f} v={v:.3f} w={w:+.3f}')
            self.last_motion=now


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--marker-id',type=int,required=True)
    p.add_argument('--move',action='store_true')
    p.add_argument('--clearance-tolerance-cm',type=float,default=1.0)
    p.add_argument('--scan-topic', default='/scan')
    args=p.parse_args()
    rclpy.init();node=Dock(
        args.marker_id,args.move,target_clearance=.10,
        clearance_tolerance=max(.002,args.clearance_tolerance_cm/100.0),
        scan_topic=args.scan_topic)
    try:
        while rclpy.ok() and node.state not in ('READY', 'COMPLETE', 'ERROR'):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f'Task C finished with state={node.state}')
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd()
        node.destroy_node()
        rclpy.try_shutdown()
if __name__=='__main__':main()

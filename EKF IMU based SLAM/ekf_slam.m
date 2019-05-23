close all
clear
clc

#load dependencies
addpath "../../"
addpath "../tools/g2o_wrapper"


function main()

    mu = zeros(6,1);
    sigma = populateSigma();
    
    #load dataset
    [sensors, landmarks, poses, observations_sframe, measurements_sframe] = loadG2o("../datasets/kalman_based_imu_slam.g2o");
    

    #computing the measurements/observations in the robot frame
    [observations, measurements] = toRobotFrame(observations_sframe, measurements_sframe, sensors);
    
    #add Euler angles to measurements
    measurements = addEuler(measurements);
    
    #mapping
    id_to_state_map = ones(10000, 1)*-1;
    state_to_id_map = ones(10000, 1)*-1;
    
    for pose = 1:length(poses)
      
      #obtain current IMU measurement
      measurement = measurements(pose);
      
      #populate Mu and Sigma 
      mu = populateMu(mu, measurement);

      #obtain current Point Sensor observartions
      observation = observations(pose);
      
      #only correct if there are observations
      if observation.pose_id!=-1
      
      #EKF correct
      [mu, sigma, id_to_state_map, state_to_id_map] = correction(mu, sigma, observation, 
                                                                        id_to_state_map, 
                                                                        state_to_id_map);
      endif
      
      #collect the trajectory
      state = collectRobotState(mu,sigma);
      trajectory(end+1,:) = state;
      
    endfor
    
    #out_trajectory(trajectory);
    #out_landmarks(mu, sigma, state_to_id_map);

end

#INPUT: observations and measurements in the sensor frames
#OUTPUT: observations and measurements in the Robot frame
function [observations, measurements] = toRobotFrame(observations_sframe, measurements_sframe, sensors)
    
    #extract the position and rotation part of sensors
    s0t = [sensors(1).x;sensors(1).y;sensors(1).z];
    s0R = toRotMat(sensors(1).qx, sensors(1).qy, sensors(1).qz, sensors(1).qw);
    s1t = [sensors(2).x;sensors(2).y;sensors(2).z];
    s1R = toRotMat(sensors(2).qx, sensors(2).qy, sensors(2).qz, sensors(2).qw);
    
    #build the transformation matrices of sensors
    s0T = buildTransMat(s0t, s0R);
    s1T = buildTransMat(s1t, s1R);

    #convert each observation to the robot frame
    for i=1:length(observations_sframe)
        for j=1:length(observations_sframe(i).observation)
            observ = observations_sframe(i).observation(j);
            mt = [observ.x;observ.y;observ.z;1];
            new_mt = s0T*mt;
            if j==1
                observations(end+1) = observation(observations_sframe(i).pose_id, observ.id, observations_sframe(i).sensor_id, new_mt(1:3));
            else
                observations(end).observation(end+1) = landmark(observ.id,new_mt(1:3));
            endif
            
        endfor
    endfor
    
    #convert each measurement to the robot frame
    for i=1:length(measurements_sframe)
        measurement = measurements_sframe(i);
        mt = [measurement.x;measurement.y;measurement.z];
        mR = toRotMat(measurement.qx, measurement.qy, measurement.qz, measurement.qw);
        mT = buildTransMat(mt, mR);
        new_mT = s1T*mT;
        [new_mt,new_mR] = decompTransMat(new_mT);
        new_quat = toQuat(new_mR);
        measurements(end+1) = pose(measurement.id, new_mt(1), new_mt(2), new_mt(3), new_quat(1), new_quat(2), new_quat(3),new_quat(4));
    endfor

end

#FUNCTIONALITY: populate mu with the measurement prior at each iteration
function mu = populateMu(mu, measurement)
    mu(1,1) = measurement.x;
    mu(2,1) = measurement.y;
    mu(3,1) = measurement.z;
    mu(4,1) = measurement.ex;
    mu(5,1) = measurement.ey;
    mu(6,1) = measurement.ez;
end
 
#FUNCTIONALITY: populate robot sigma (fixed for all iterations)
function sigma = populateSigma()
    sigma = zeros(6,6);
    sigma(1,1) = 1000;
    sigma(2,2) = 1000;
    sigma(3,3) = 10;
    sigma(4,4) = 1000;
    sigma(5,5) = 1000;
    sigma(6,6) = 1000;
end


#INPUT: position t and Rotation matrix R
#OUTPUT: transformation matrix T
function T = buildTransMat(t, R)
    T = zeros(4,4);
    T(1:3,4) = t;
    T(1:3,1:3) = R;
    T(4,:) = [0, 0, 0, 1];
end

#INPUT: quaternion 
#OUTPUT: Rotation Matrix
function R = toRotMat(x, y, z, w)
    R = [w**2+x**2-y**2-z**2,   2*x*y-2*w*z,    2*x*z+2*w*y;
         2*x*y+2*w*z,   w**2-x**2+y**2-z**2,    2*y*z-2*w*x;
         2*x*z-2*w*y,   2*y*z+2*w*x,    w**2-x**2-y**2+z**2]; 
end

#INPUT: transformation matrix T
#OUTPUT: position t and Rotation matrix R
function [t,R] = decompTransMat(T)
    t = T(1:3,4);
    R = T(1:3,1:3);
end

#INPUT: Rotation Matrix
#OUTPUT: Quaternion
function q = toQuat(R)
    tr = R(1,1)+R(2,2)+R(3,3);
    if tr<=-1
        disp("not valid");
        disp(tr);
    endif
    m00=R(1,1); m01=R(1,2); m02=R(1,3);
    m10=R(2,1); m11=R(2,2); m12=R(2,3);
    m20=R(3,1); m21=R(3,2); m22=R(3,3);
    w = sqrt(1.0 + m00 + m11 + m22) / 2.0;
	w4 = (4.0 * w);
	x = (m21 - m12) / w4 ;
	y = (m02 - m20) / w4 ;
	z = (m10 - m01) / w4 ;
	q = [x;y;z;w];
end

#INPUT: measurements 
#OUTPUT: measurement with added Euler angles
function measurements = addEuler(measurements)

    for i=1:length(measurements)
        [ex, ey, ez] = toEuler(measurements(i).qx,measurements(i).qy,measurements(i).qz,measurements(i).qw);
        measurements(i).ex = ex;
        measurements(i).ey = ey;
        measurements(i).ez = ez;
    endfor

end

#INPUT: Quaternion
#OUTPUT: Euler
function [X, Y, Z] = toEuler(x,y,z,w)

    ysqr = y * y;
    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + ysqr);
    X = atan2(t0, t1);

	t2 = 2.0 * (w * y - z * x);
    if t2<-1
        t2=-1;
    elseif t2>1
        t2=1;
    endif

    Y = asin(t2);

    t3 = 2.0 * (w * z + x * y);
    t4 = 1.0 - 2.0 * (ysqr + z * z);
    Z = atan2(t3, t4);
    
end

#INPUT: Euler
#OUTPUT: Quaternion
function [x,y,z,w] = getQuat(pitch, roll, yaw)
    cy = cos(yaw * 0.5);
	sy = sin(yaw * 0.5);
	cr = cos(roll * 0.5);
	sr = sin(roll * 0.5);
	cp = cos(pitch * 0.5);
	sp = sin(pitch * 0.5);

	w = cy * cr * cp + sy * sr * sp;
	x = cy * sr * cp - sy * cr * sp;
	y = cy * cr * sp + sy * sr * cp;
	z = sy * cr * cp - cy * sr * sp;
end


#INPUT: Euler
#OUTPUT: Rotation Matrix
function R = toRotMat2(mu_ex, mu_ey, mu_ez)
  cx = cos(mu_ex);
  sx = sin(mu_ex);
  cy = cos(mu_ey);
  sy = sin(mu_ey);
  cz = cos(mu_ez);
  sz = sin(mu_ez);
  
  # re precompute some quantities that come in handy later on
  R   = [cy*cz, -cx*sz+sx*sy*cz, sx*sz+cx*sy*cz;
         cy*sz, cx*cz+sx*sy*sz, -sx*cz+cx*sy*sz;
         -sy, sx*cy, cx*cy;];
end

#INPUT: mu, sigma
#OUTPUT: Robot pose and covariance
function state = collectRobotState(mu,sigma)
      state = mu(1:3)';
      [qx,qy,qz,qw] = getQuat(mu(4),mu(5),mu(6));
      state(:,end+1) = qx;
      state(:,end+1) = qy;
      state(:,end+1) = qz;
      state(:,end+1) = qw;    
      for j=1:6
          state(:,end+1) = sigma(j,j);
      endfor
end

#FUNCTIONALITY: saves the estimated landmarks to an output file
function out_landmarks(mu, sigma, state_to_id_map)
    filename = "landmarks.txt";
    fid = fopen (filename, "w");
    fdisp(fid, "id\t\tx\t\ty\t\tz\t\tcovariance(major diagonal)\n\n");
    n = (length(mu)-6)/3;
    ind = 7;
    for i=1:n
        id = state_to_id_map(i);
        fprintf(fid, "%d\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\n", id, mu(ind), mu(ind+1), mu(ind+2),
                                                                        sigma(ind,ind), sigma(ind+1,ind+1), sigma(ind+2, ind+2));
        ind = ind + 3;
    endfor
    fclose (fid);
end

#FUNCTIONALITY: saves the estimated trajectory to an output file
function out_trajectory(t)
    disp(size(t));
    filename = "trajectory.txt";
    fid = fopen (filename, "w");
    fdisp(fid, "id\t\tx\t\ty\t\tz\t\tqx\t\tqy\t\tqz\t\tqw\t\t\t\t\tcovariance(major diagonal)\n\n");
    for i=1:length(t)
        fprintf(fid, "%d\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n",
                    i+1000,t(i,1),t(i,2),t(i,3),t(i,4),t(i,5),t(i,6),t(i,7),t(i,8),t(i,9),t(i,10),t(i,11),t(i,12),t(i,13));
    endfor
    fclose(fid);
end

main();


#this function computes the update (also called correction) step of the filter
#inputs:
#  mu: mean, 
#  sigma: covariance of the robot-landmark set
#
#  observations:
#            a structure containing n observations of landmarks
#            for each observation we have
#            - the index of the landmark seen
#            - the location where we have seen the landmark (x,y,z) w.r.t the robot
#
#  id_to_state_map:
#            mapping that given the id of the measurement, returns its position in the mu vector
#  state_to_id_map:
#            mapping that given the index of mu vector, returns the id of the respective landmark
#
#outputs:
#  [mu, sigma]: the updated mean and covariance
#  [id_to_state_map, state_to_id_map]: the updated mapping vector between landmark position in mu vector and its id


function [mu, sigma, id_to_state_map, state_to_id_map] = correction(mu, sigma, observations, id_to_state_map, state_to_id_map)
 
  #determine how many landmarks we have seen in this step
  num_landmarks_measured = length(observations.observation);
  
  #dimension of the state (robot pose + landmark positions)
  state_dim = size(mu,1);
  
  #dimension of the current map (how many landmarks we have already seen)
  num_landmarks = (state_dim-6)/3;
  
  #if I've seen no landmarks, i do nothing
  if (num_landmarks_measured == 0)
    disp("no ob");
    return;
  endif

  mu_t  = mu(1:3,:); # translational part of the robot pose
  mu_ex = mu(4);
  mu_ey = mu(5);
  mu_ez = mu(6);
     
  cx = cos(mu_ex);
  sx = sin(mu_ex);
  cy = cos(mu_ey);
  sy = sin(mu_ey);
  cz = cos(mu_ez);
  sz = sin(mu_ez);
  
  # re precompute some quantities that come in handy later on
  R   = [cy*cz, -cx*sz+sx*sy*cz, sx*sz+cx*sy*cz;
         cy*sz, cx*cz+sx*sy*sz, -sx*cz+cx*sy*sz;
         -sy, sx*cy, cx*cy];
  Rt = R';

  #partial derivatives of rotation matrix
  
  Rpx = [0,  sz*sx+cx*sy*cz, cx*sz-sx*sy*cz;
         0, -sx*cz+cx*sy*sz, -cx*cz-sx*sy*sz;
         0, cx*cy, -sx*cy];
    
  Rpy = [-sy*cz, sx*cy*cz, cx*cy*cz;
         -sy*sz, sx*cy*sz, cx*cy*sz;
         -cy, -sx*sy, -cx*sy];
  
  Rpz = [-cy*sz, -cx*cz-sx*sy*sz, sx*cz-cx*sy*sz;
         cy*cz, -cx*sz+sx*sy*cz, sx*sz+cx*sy*cz;
         0, 0, 0];
 
  Rtpx = Rpx';
  Rtpy = Rpy';
  Rtpz = Rpz';
  # for below computation, we need to count how many observations 
  # of old landmarks we have
  num_old_landmarks_measured = 0;

  # Here two cases arise, the current landmark has been already seen, i.e. REOBSERVED landmark,
  # or the current landmark is completely new, i.e. NEW landmark.
  #
  # for simplicity we can divide the problem: first analyze only the reobserved landmark
  # and work with them as in a localization procedure (of course, with full Jacobian now).
  # With this reobserved landmark we compute the correction/update of mean and covariance.
  # Then, after the correction is done, we can simply add the new landmark expanding the
  # mean and the covariance.
  #
  #
  # First of all we are interested in REOBSERVED landmark 
  for i=1:num_landmarks_measured
  
    #retrieve info about the observed landmark
    measurement = observations.observation(i);

    #fetch the position in the state vector corresponding to the actual measurement
    state_pos_of_landmark = id_to_state_map(measurement.id);

    #compute the index (vector coordinate) in the state vector corresponding to the pose of the landmark;	
    landmark_state_vector_index = 6+3*(state_pos_of_landmark-1)+1;

    #IF current landmark is a REOBSERVED LANDMARK
    if(state_pos_of_landmark != -1) 

      #increment the counter of observations originating
      #from already known landmarks
      num_old_landmarks_measured++;

      # where we see the landmark
      z_t(end+1,:) = measurement.x; 
      z_t(end+1,:) = measurement.y;
      z_t(end+1,:) = measurement.z;

      #fetch the position of the landmark in the state
      landmark_mu=mu(landmark_state_vector_index:landmark_state_vector_index+2,:);
      
      #where I predict i will see that landmark
      delta_t            = landmark_mu-mu_t;
      measure_prediction = Rt * delta_t;

      #add prediction to prediction vector
      h_t(end+1,:) = measure_prediction(1);
      h_t(end+1,:) = measure_prediction(2);
      h_t(end+1,:) = measure_prediction(3);

      #jacobian w.r.t robot
      C_m=zeros(3,state_dim);
      C_m(1:3,1:3) = -Rt;
      C_m(1:3,4)   = Rtpx*delta_t;
      C_m(1:3,5)   = Rtpy*delta_t;
      C_m(1:3,6)   = Rtpz*delta_t;

      #jacobian w.r.t landmark
      C_m(:,landmark_state_vector_index:landmark_state_vector_index+2)=Rt;
      
      C_t(end+1,:) = C_m(1,:);
      C_t(end+1,:) = C_m(2,:);
      C_t(end+1,:) = C_m(3,:);
    endif
  endfor

  #if I have seen again at least one landmark
  #I need to update, otherwise I jump to the new landmark case
  if ((num_old_landmarks_measured > 0))

    #observation noise
    sigma_z = eye(3*num_old_landmarks_measured)*1000;
    for i=1:3*num_old_landmarks_measured
        if rem(i,3) == 0
            sigma_z(i,i) = 10;
        endif
    endfor
    #Kalman gain
    K = sigma * C_t' * (inv(C_t*sigma*C_t' + sigma_z));

    #update mu
    error      = (z_t - h_t);
    correction = K*error;
    mu         = mu + correction;

    #update sigma
    sigma = (eye(state_dim) - K*C_t)*sigma;		
  endif

  #since I have applied the correction, I need to update my
  #data with the new mu values
  mu_t     = mu(1:3,:); #translational part of the robot pose
  mu_x = mu(4);
  mu_y = mu(5);
  mu_z = mu(6);

  #from Euler to Quaternion
  [x,y,z,w] = getQuat(mu_ex,mu_ey,mu_ez);
  
  # re precompute some quantities that come in handy later on
  R   = [w**2+x**2-y**2-z**2,   2*x*y-2*w*z,    2*x*z+2*w*y;
         2*x*y+2*w*z,   w**2-x**2+y**2-z**2,    2*y*z-2*w*x;
         2*x*z-2*w*y,   2*y*z+2*w*x,    w**2-x**2-y**2+z**2];  #rotation matrix
  
  Rt = R';

  
  #Now its time to add, if observed, the NEW landmaks, without applying any correction
  for i=1:num_landmarks_measured

    #retrieve info about the observed landmark
    measurement = observations.observation(i);

    #fetch the position in the state vector corresponding to the actual measurement
    state_pos_of_landmark=id_to_state_map(measurement.id);

    #IF current landmark is a NEW landmark
    if(state_pos_of_landmark == -1) 

      #adjust direct and reverse mappings
      num_landmarks++;
      id_to_state_map(measurement.id)=num_landmarks;
      state_to_id_map(num_landmarks)=measurement.id;
      
      #landmark position in the world
      land_pose_world = mu_t + R*[measurement.x;measurement.y;measurement.z];

      #retrieve from the index the position of the landmark block in the state
      new_landmark_state_vector_index=6+3*(num_landmarks-1)+1;
 
      #increase mu and sigma size
      mu(new_landmark_state_vector_index:new_landmark_state_vector_index+2,1) = land_pose_world;

      #initial noise assigned to a new landmark
      landmark_sigma = zeros(3,3);
      landmark_sigma(1,1)=1000; landmark_sigma(2,2)=1000; landmark_sigma(3,3)=10;
    

      #extend the structure
      sigma(new_landmark_state_vector_index,:)   = 0;
      sigma(new_landmark_state_vector_index+1,:) = 0;
      sigma(new_landmark_state_vector_index+2,:) = 0;
      sigma(:,new_landmark_state_vector_index)   = 0;
      sigma(:,new_landmark_state_vector_index+1) = 0;
      sigma(:,new_landmark_state_vector_index+2) = 0;

      #add the covariance block
      sigma(new_landmark_state_vector_index:new_landmark_state_vector_index+2,
	  new_landmark_state_vector_index:new_landmark_state_vector_index+2)=landmark_sigma;

    endif
  endfor
end

#INPUT: Euler
#OUTPUT: Quaternion
function [x,y,z,w] = getQuat(pitch,roll,yaw)
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

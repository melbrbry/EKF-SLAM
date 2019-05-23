% #   This source code is part of the localization and SLAM package
% #   deveoped for the lectures of probabilistic robotics at 
% #   Sapienza, University of Rome.
% #  
% #     Copyright (c) 2016 Bartolomeo Della Corte, Giorgio Grisetti
% #  
% #   It is licences under the Common Creative License,
% #   Attribution-NonCommercial-ShareAlike 3.0
% #  
% #   You are free:
% #     - to Share - to copy, distribute and transmit the work
% #     - to Remix - to adapt the work
% #  
% #   Under the following conditions:
% #  
% #     - Attribution. You must attribute the work in the manner specified
% #       by the author or licensor (but not in any way that suggests that
% #       they endorse you or your use of the work).
% #    
% #     - Noncommercial. You may not use this work for commercial purposes.
% #    
% #     - Share Alike. If you alter, transform, or build upon this work,
% #       you may distribute the resulting work only under the same or
% #       similar license to this one.
% #  
% #   Any of the above conditions can be waived if you get permission
% #   from the copyright holder.  Nothing in this license impairs or
% #   restricts the author's moral rights.
% #  
% #   This software is distributed in the hope that it will be useful,
% #   but WITHOUT ANY WARRANTY; without even the implied 
% #   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% #   PURPOSE.
% #
%   
% # load a file.g2o file and returns the four structs of landmark, poses, transitions, observations

function [sensors, landmarks, poses, observations, measurements] = loadG2o(filepath)

	%%-----G2O specification---
	PARAMS_SE3OFFSET = 'PARAMS_SE3OFFSET'; #sensors poses
    VERTEX_TRACKXYZ = 'VERTEX_TRACKXYZ'; #landmarks positions
	VERTEX_SE3 = 'VERTEX_SE3:QUAT'; #robot poses
	EDGE_SE3_TRACKXYZ = 'EDGE_SE3_TRACKXYZ'; #point sensor observations
	EDGE_SE3_PRIOR = 'EDGE_SE3_PRIOR'; #IMU measuerments
	%%-------------------------

	%open the file
	fid = fopen(filepath, 'r');
	

	%debug stuff
	i_sen_se3 = 0;
	i_vert_xyz = 0;
	i_vert_se3 = 0;
	i_track_se3 = 0;
	i_prior_se3= 0;
	%    
	curr_id = -1;

	while true
		%get current line
		c_line = fgetl(fid);

		%stop if EOF
		if c_line == -1
			break;
		end

		%Split the line using space as separator
		elements = strsplit(c_line,' ');

		switch(elements{1})
		    case PARAMS_SE3OFFSET
        		sensors(end+1) = extractPose(elements);
				i_sen_se3 = i_sen_se3 + 1; 
			case VERTEX_TRACKXYZ
        		landmarks(end+1) = extractLandmark(elements);
				i_vert_xyz = i_vert_xyz + 1; %do not use pre/post increment. Keep the Matlab compatibility
			case VERTEX_SE3
        		poses(end+1) = extractPose(elements);
				i_vert_se3 = i_vert_se3 + 1;
			case EDGE_SE3_TRACKXYZ
				current_obs = extractObservation(elements);				
				if current_obs.pose_id == curr_id
					observations(end).observation(end+1) = current_obs.observation;
				else
				    if curr_id != -1
    				    for i=1:(current_obs.pose_id-curr_id-1)
    				        observations(end+1) = observation(-1,-1,-1, [-1; -1; -1]);
				        endfor
				     endif
				    observations(end+1) = current_obs;
					curr_id = observations(end).pose_id;
					i_track_se3 = i_track_se3 + 1;
				endif
				
			case EDGE_SE3_PRIOR
			    measurements(end+1) = extractMeasurement(elements);
				i_prior_se3 = i_prior_se3 + 1;
			otherwise
				disp('Error in reading element');
				
		end
	end
  
  printf('[G2oWrapper] loading file...\n');
  printf('#sensors: %d \n', i_sen_se3);
  printf('#landmarks: %d \n#poses: %d \n',i_vert_xyz, i_vert_se3);
  printf('#observations: %d \n#measurements: %d \n',i_track_se3, i_prior_se3);  
  fflush(stdout);

end

function out = extractLandmark(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  z_pose = str2double(elements{5});
  out = landmark(id,[x_pose,y_pose,z_pose]);
end

function out = extractPose(elements)
  id = str2double(elements{2});
  x_pose = str2double(elements{3});
  y_pose = str2double(elements{4});
  z_pose = str2double(elements{5});
  qx_pose = str2double(elements{6});
  qy_pose = str2double(elements{7});
  qz_pose = str2double(elements{8});
  qw_pose = str2double(elements{9});
  out = pose(id,x_pose, y_pose, z_pose, qx_pose, qy_pose, qz_pose, qw_pose);
end

function out = extractObservation(elements)
  from_id = str2double(elements{2});
  land_id = str2double(elements{3});
  sensor_id = str2double(elements{4});
  x_p = str2double(elements{5});
  y_p = str2double(elements{6});
  z_p = str2double(elements{7});
  out = observation(from_id,land_id,sensor_id, [x_p; y_p; z_p]);
end

function out = extractMeasurement(elements)
  from_id = str2double(elements{2});
  x_pose = str2double(elements{4});
  y_pose = str2double(elements{5});
  z_pose = str2double(elements{6});
  qx_pose = str2double(elements{7});
  qy_pose = str2double(elements{8});
  qz_pose = str2double(elements{9});
  qw_pose = str2double(elements{10});
  out = pose(from_id,x_pose, y_pose, z_pose, qx_pose, qy_pose, qz_pose, qw_pose);
end

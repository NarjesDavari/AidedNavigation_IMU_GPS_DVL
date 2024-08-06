function [Simulation]=R_adaptive(Simulation,R,N,update_Counter,I,Selection_Param)

global GPS_fusion_active ;
global depth_fusion_active ;
%  global DVL_fusion_active ;
global Hdng_fusion_active ;
global R_DVL_active;
global rollpitch_fusion_active;
global accelrollpitch;
time_IMU = Selection_Param{1};
time_GPS = Selection_Param{2};
time_Depth = Selection_Param{3};
time_DVL   = Selection_Param{4};
time_rollpitch = 0;
time_Heading  = Selection_Param{5};
include_GPS = Selection_Param{6};
include_depthmeter = Selection_Param{7};
include_dvl = Selection_Param{8};
include_heading = Selection_Param{9};
include_rollpitch = 0;
include_accelrollpitch = Selection_Param{10};


if include_GPS && ~accelrollpitch
%     if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
       if GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
           Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
          Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
       end
%     end
end
 

if include_dvl && ~accelrollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && ~depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(1,1);
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(2,2);
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(3,3);
        end
%     end
end

if include_depthmeter && ~accelrollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
        end
%     end
end 


% if include_rollpitch && ~accelrollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==0 && time_Depth==0 && time_rollpitch==1 && time_Heading==0
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(1,1);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(2,2);
%     end
% end

if include_heading && ~accelrollpitch
%      if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
         if ~GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
             Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
             Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R;
             
         end
%      end
end

if accelrollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(2,2);
        end
%     end
end

%%%%%%%%%%%%%%%%%%%%
% if ~accelrollpitch && include_depthmeter && include_rollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==0 && time_Depth==1 && time_rollpitch==1 && time_Heading==0
%  Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%         Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
%         Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(2,2);   
%         Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(3,3); 
%     end
% end
if ~accelrollpitch && include_depthmeter && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if ~GPS_fusion_active && depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
          Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(2,2);
       end
%     end
end
% if ~accelrollpitch && include_rollpitch && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==0 && time_Depth==0 && time_rollpitch==1 && time_Heading==1
%         Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(1,1);
%         Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(2,2);
%         Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%         Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(3,3);
%     end
% end
if ~accelrollpitch && include_depthmeter && include_dvl
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% if time_GPS==0 && time_DVL==1 && time_Depth==1 && time_rollpitch==0 && time_Heading==0
    if ~GPS_fusion_active && depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
       Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
       Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
       Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
       Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(2,2);
       Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(3,3);
       Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(4,4);
    end
end
% if ~accelrollpitch && include_dvl && include_rollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==1 && time_Depth==0 && time_rollpitch==1 && time_Heading==0
% Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(1,1);
%        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(2,2);
%        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(3,3);
%        Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(4,4);
%        Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(5,5);
%     end
% end
if ~accelrollpitch && include_dvl && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if ~GPS_fusion_active && ~depth_fusion_active && R_DVL_active && Hdng_fusion_active
           Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(1,1);
        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(2,2);
        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(3,3);
        Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(4,4);
       end
%     end
end
if ~accelrollpitch && include_GPS && include_depthmeter
%     if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
       if GPS_fusion_active && depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
           Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
          Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
           Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
       end
%     end
end
if ~accelrollpitch && include_GPS && include_dvl 
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
      if GPS_fusion_active && ~depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
          Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
      Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
      Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
      Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
      Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(3,3);
      Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(4,4);
      Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(5,5);
      end
%     end
end
% if ~accelrollpitch && include_GPS &&  include_rollpitch
%     if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
%         Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
%            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%            Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(3,3);
%            Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(4,4);
%     end
% end
if ~accelrollpitch && include_GPS && include_heading
%     if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(3,3);
        end
%     end
end

if accelrollpitch && include_GPS
%     if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
        Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
        Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(3,3);
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(4,4);
        end
%     end
end
if accelrollpitch && include_dvl 
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && ~depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(1,1);
        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(2,2);
        Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(3,3);
        Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(4,4);
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(5,5);
        end
%     end
end
if accelrollpitch && include_depthmeter
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1); 
           Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(2,2);
           Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(3,3);
        end
%     end
end
if accelrollpitch && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if ~GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
           Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(1,1);
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(2,2);
        Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(3,3);
       end
%     end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if ~accelrollpitch &&  include_depthmeter && include_rollpitch && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==0 && time_Depth==1 && time_rollpitch==1 && time_Heading==1
%  Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(2,2);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(3,3);
%     Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(4,4);
%     end
% end

% if ~accelrollpitch &&  include_depthmeter && include_dvl && include_rollpitch
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==1 && time_Depth==1 && time_rollpitch==1 && time_Heading==0
%  Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
%     Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(2,2);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(3,3);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(4,4);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(5,5);   
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(6,6); 
%     end
% end

if ~accelrollpitch &&  include_depthmeter && include_dvl && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if ~GPS_fusion_active && depth_fusion_active && R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
          Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(2,2);
          Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(3,3);
          Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(4,4);
          Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
          Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(5,5);
       end
%     end
end

% if ~accelrollpitch && include_dvl && include_rollpitch && include_heading
%     if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==1 && time_Depth==0 && time_rollpitch==1 && time_Heading==1
% Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(1,1);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(2,2);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(3,3);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(4,4);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(5,5);
%     Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(6,6);    
%     end
% end

if ~accelrollpitch &&  include_GPS && include_depthmeter && include_dvl
%    if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
       if GPS_fusion_active && depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
           Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
           Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
            Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
           Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(4,4);
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(5,5);
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(6,6);
       end
%    end
end


% if ~accelrollpitch &&  include_GPS && include_depthmeter && include_rollpitch
%    if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% % if time_GPS==1 && time_DVL==0  && time_Depth==1 && time_rollpitch==1 && time_Heading==0
% Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%     Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(4,4);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(5,5);
%    end
% end

if ~accelrollpitch &&  include_GPS && include_depthmeter && include_heading
%    if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if GPS_fusion_active && depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
           Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
           Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
            Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
           Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(4,4);
       end
%    end
end

% if ~accelrollpitch &&  include_GPS  && include_dvl && include_rollpitch
%    if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% %  if time_GPS==1 && time_DVL==1  && time_Depth==0 && time_rollpitch==1 && time_Heading==0
% Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%    Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(3,3);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(4,4);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(5,5);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(6,6);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(7,7); 
%    end
% end

if ~accelrollpitch &&  include_GPS  && include_dvl && include_heading
%    if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if GPS_fusion_active && ~depth_fusion_active && R_DVL_active && Hdng_fusion_active
           Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
           Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
           Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(3,3);
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(4,4);
           Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(5,5);
           Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
           Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(6,6);
       end
%    end
end

% if ~accelrollpitch &&  include_GPS  && include_rollpitch && include_heading 
%    if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% %  if time_GPS==1 && time_DVL==0  && time_Depth==0 && time_rollpitch==1 && time_Heading==1
% Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(3,3);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(4,4); 
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(5,5); 
%    end
% end
 
if accelrollpitch &&  include_GPS && include_dvl 
%     if strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && ~depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(3,3);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(5,5);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(6,6);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(7,7);
        end
%     end
end

if accelrollpitch &&  include_GPS && include_depthmeter
%     if strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && depth_fusion_active && ~R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(5,5);
        end
%     end
end
  
if accelrollpitch &&  include_GPS && include_heading 
%     if strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && ~depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(3,3);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(4,4);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(5,5);
        end
%     end
end

if accelrollpitch && include_dvl && include_depthmeter
%     if ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(2,2);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(3,3);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(4,4);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(5,5);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(6,6);
        end
%     end
end

if accelrollpitch && include_depthmeter && include_heading
%     if ~strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
       if ~GPS_fusion_active && depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
        Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(2,2);
        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(3,3);
        Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
        Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(4,4);
       end
%     end
end

if accelrollpitch && include_dvl && include_heading
%     if ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && ~depth_fusion_active && R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(2,2);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(3,3);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(5,5);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(6,6);
        end
%     end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if ~accelrollpitch && include_dvl && include_depthmeter && include_rollpitch && include_heading
%    if  ~strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% % if time_GPS==0 && time_DVL==1 && time_Depth==1 && time_rollpitch==1 && time_Heading==1
%  Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
%     Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(2,2);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(3,3);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(4,4); 
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(5,5);
%     Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(6,6);
%     Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(7,7);
%    end
% end

% if ~accelrollpitch && include_GPS && include_dvl && include_depthmeter && include_rollpitch
%    if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
% % if time_GPS==1 && time_DVL==1 && time_Depth==1 && time_rollpitch==1 && time_Heading==0
% Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%     Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
%    Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(4,4);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(5,5);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(6,6);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(7,7);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(8,8);
%    end
% end

if ~accelrollpitch && include_GPS && include_dvl && include_depthmeter && include_heading
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && depth_fusion_active && R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(5,5);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(6,6);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(7,7);
        end
%     end
end

% if ~accelrollpitch && include_GPS && include_dvl && include_rollpitch && include_heading
%    if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && ~strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% % if time_GPS==1 && time_DVL==1 && time_Depth==0 && time_rollpitch==1 && time_Heading==1
% Simulation.Input.Measurements.GPS_Counter_R_R=Simulation.Input.Measurements.GPS_Counter_R_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%    Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(3,3);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(4,4);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(5,5);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(6,6);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(7,7);
%    Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(8,8); 
%    end
% end
  
% if ~accelrollpitch && include_GPS  && include_depthmeter && include_rollpitch && include_heading
%    if  strcmp(time_IMU,time_GPS)  && ~strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% %   if time_GPS==1 && time_DVL==0 && time_Depth==1 && time_rollpitch==1 && time_Heading==1
% Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
%     Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%     Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(4,4);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(5,5);
%    Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(6,6); 
%    end
% end

if accelrollpitch && include_GPS  && include_depthmeter && include_dvl
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && depth_fusion_active && R_DVL_active && ~Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(5,5);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(6,6);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(7,7);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(8,8);
        end
%     end
end
  
if accelrollpitch && include_GPS && include_dvl && include_heading
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && ~depth_fusion_active && R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(3,3);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(5,5);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(6,6);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(7,7);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(8,8);
        end
%     end
end

if accelrollpitch && include_GPS  && include_depthmeter && include_heading
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if GPS_fusion_active && depth_fusion_active && ~R_DVL_active && Hdng_fusion_active
            Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(4,4);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(5,5);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(6,6);
        end
%     end
end

if accelrollpitch && include_depthmeter && include_dvl && include_heading
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && ~strcmp(time_IMU,time_Heading)
        if ~GPS_fusion_active && depth_fusion_active && R_DVL_active && Hdng_fusion_active
             Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(1,1);
            Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(2,2);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(3,3);
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(4,4);
            Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(5,5);
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(6,6);
            Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(7,7);
        end
%     end
end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  if ~accelrollpitch && include_GPS && include_depthmeter && include_dvl && include_rollpitch && include_heading
%     if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
% %  if time_GPS==1 && time_DVL==1 && time_Depth==1 && time_rollpitch==1 && time_Heading==1
% Simulation.Input.Measurements.GPS_Counter_R_R=Simulation.Input.Measurements.GPS_Counter_R_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
%     Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
%    Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(4,4);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(5,5);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(6,6);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,1)=R(7,7);
%    Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch(Simulation.Input.Measurements.incln_Counter,2)=R(8,8);
%    Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
%    Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(9,9); 
%     end
%  end

 
 if accelrollpitch && include_GPS && include_depthmeter && include_dvl &&  include_heading
%      if  strcmp(time_IMU,time_GPS)  && strcmp(time_IMU,time_DVL) && strcmp(time_IMU,time_Depth) && ~strcmp(time_IMU,time_rollpitch) && strcmp(time_IMU,time_Heading)
         if GPS_fusion_active && depth_fusion_active && R_DVL_active && Hdng_fusion_active
             Simulation.Input.Measurements.GPS_Counter_R=Simulation.Input.Measurements.GPS_Counter_R+1;
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,1)=R(1,1);
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter_R,2)=R(2,2);
              Simulation.Input.Measurements.Depth_Counter_R=Simulation.Input.Measurements.Depth_Counter_R+1;
             Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter_R,1)=R(3,3);
             Simulation.Input.Measurements.DVL_Counter_R=Simulation.Input.Measurements.DVL_Counter_R+1;
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,1)=R(4,4);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,2)=R(5,5);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter_R,3)=R(6,6);
             Simulation.Input.Measurements.accelrollpitch_Counter_R= Simulation.Input.Measurements.accelrollpitch_Counter_R+1;
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,1)=R(7,7);
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter_R,2)=R(8,8);
             Simulation.Input.Measurements.hdng_Counter_R=Simulation.Input.Measurements.hdng_Counter_R+1;
             Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter_R,1)=R(9,9);
         end
%      end
 end
 


 
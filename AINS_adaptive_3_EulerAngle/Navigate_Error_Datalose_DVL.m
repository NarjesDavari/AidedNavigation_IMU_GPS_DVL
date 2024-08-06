function [ Simulation] = Navigate_Error_Datalose_DVL ( Simulation,ave_sample  )
%     N = GetParam(Simulation.Init_Value ,'simulation_number');
% t1=[118049 136049 154049  172049  190049  208049];%test7
% t2=[119057 137057  155077  173057 191057  209057];%test7

% t1=[219841  239841   259841  279841  299841  319841]; %test3
% t2=[220849  240849  260849   280849  300849  320849]; %test3
t1=Simulation.Output.ESKF.Pos_Error.DVL_Outage.t1;
t2=Simulation.Output.ESKF.Pos_Error.DVL_Outage.t2;

            %Creation of space in memory for relative and absolute error
%             Simulation.Output.ESKF.Pos_Error.relative_error_i=zeros(Dlength-1,N);
%             Simulation.Output.ESKF.Pos_Error.absolute_error_i=zeros(Dlength,N); 
              Simulation.Output.ESKF.Pos_Error.DVL_Outage.relative_RMSE_loosData=zeros(length(t1),3);
              Simulation.Output.ESKF.Pos_Error.DVL_Outage.RMSE_i_loosData=zeros(length(t1),3);
%             Simulation.Output.ESKF.Pos_Error.RMSEx_i=zeros(N,1);
%             Simulation.Output.ESKF.Pos_Error.RMSEy_i=zeros(N,1);
%             Simulation.Output.ESKF.Pos_Error.RMSEz_i=zeros(N,1);            
            
            %inserting of travelled time and distance in memory
             
s=zeros(1,3);
for i=1:length(t1)
    [Simulation,rela_RMSE,RMSE_i,RMSE_outage,absolute_error_outage,absolute_error_end]=CalculError(Simulation,t1(i),t2(i),ave_sample );
    Simulation.Output.ESKF.Pos_Error.DVL_Outage.relative_RMSE_loosData(i,:)=rela_RMSE;
    Simulation.Output.ESKF.Pos_Error.DVL_Outage.RMSE_i_loosData(i,:)=RMSE_i;
    Simulation.Output.ESKF.Pos_Error.DVL_Outage.RMSE_outage(i,1)=RMSE_outage;
%     Simulation.Output.ESKF.Pos_Error.DVL_Outage.absolute_error_outage(:,i)=absolute_error_outage;
    Simulation.Output.ESKF.Pos_Error.DVL_Outage.absolute_error_end(i,1)=absolute_error_end;
    rmse(i,:)=RMSE_i;
    s(1,:)=s(1,:)+rmse(i,:).^2;
end

 Simulation.Output.ESKF.Pos_Error.DVL_Outage.RMSE_total=sqrt(s(1,:)/length(t1));
 Simulation.Output.ESKF.Pos_Error.DVL_Outage.mean_RMSE_outage=mean(Simulation.Output.ESKF.Pos_Error.DVL_Outage.RMSE_outage(:));
 Simulation.Output.ESKF.Pos_Error.DVL_Outage.mean_absolute_error_end=mean(Simulation.Output.ESKF.Pos_Error.DVL_Outage.absolute_error_end(:));


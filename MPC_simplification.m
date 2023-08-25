function [Hdb,Fdbt,Cdb,Adc,C_cm_g,y_max_global,y_min_global] = MPC_simplification(Ad,Bd,Cd,Dd,hz,y_max,y_min)
    
    % db - double bar
    % dbt - double bar transpose
    % dc - double circumflex
    
    A_aug=[Ad,Bd;zeros(length(Bd(1,:)),length(Ad(1,:))),eye(length(Bd(1,:)))];
    B_aug=[Bd;eye(length(Bd(1,:)))];
    C_aug=[Cd,zeros(length(Cd(:,1)),length(Bd(1,:)))];
    D_aug=Dd; % D_aug is not used because it is a zero matrix
    
    constants = initial_constants();
    Q = constants('Q');
    S = constants('S');
    R  = constants('R');
    
    %% Constraints
    C_cm=constants('C_cm');
    C_cm_size=size(C_cm);
    C_cm_g=zeros(C_cm_size(1)*hz,C_cm_size(2)*hz);
    y_max_global=zeros(length(y_max)*hz,1);
    y_min_global=zeros(length(y_min)*hz,1);
    
    CQC=C_aug'*Q*C_aug;
    CSC=C_aug'*S*C_aug;
    QC=Q*C_aug;
    SC=S*C_aug;
    
    Qdb=zeros(length(CQC(:,1))*hz,length(CQC(1,:))*hz);
    Tdb=zeros(length(QC(:,1))*hz,length(QC(1,:))*hz);
    Rdb=zeros(length(R(:,1))*hz,length(R(1,:))*hz);
    Cdb=zeros(length(B_aug(:,1))*hz,length(B_aug(1,:))*hz);
    Adc=zeros(length(A_aug(:,1))*hz,length(A_aug(1,:)));
    
    for i = 1:hz
        
       if i == hz
           Qdb(1+length(CSC(:,1))*(i-1):length(CSC(:,1))*i,1+length(CSC(1,:))*(i-1):length(CSC(1,:))*i)=CSC;
           Tdb(1+length(SC(:,1))*(i-1):length(SC(:,1))*i,1+length(SC(1,:))*(i-1):length(SC(1,:))*i)=SC;           
           %% Constraints
           C_cm_g(C_cm_size(1)*(i-1)+1:C_cm_size(1)*(i-1)+C_cm_size(1), ...
               C_cm_size(2)*(i-1)+1:C_cm_size(2)*(i-1)+C_cm_size(2))=C_cm;           
       else
           Qdb(1+length(CQC(:,1))*(i-1):length(CQC(:,1))*i,1+length(CQC(1,:))*(i-1):length(CQC(1,:))*i)=CQC;
           Tdb(1+length(QC(:,1))*(i-1):length(QC(:,1))*i,1+length(QC(1,:))*(i-1):length(QC(1,:))*i)=QC;
           %% Constraints
           C_cm_g(C_cm_size(1)*(i-1)+1:C_cm_size(1)*(i-1)+C_cm_size(1), ...
               C_cm_size(2)*(i-1)+1:C_cm_size(2)*(i-1)+C_cm_size(2))=C_cm;
       end
       
       Rdb(1+length(R(:,1))*(i-1):length(R(:,1))*i,1+length(R(1,:))*(i-1):length(R(1,:))*i)=R;
       
       for j = 1:hz
           if j<=i
               Cdb(1+length(B_aug(:,1))*(i-1):length(B_aug(:,1))*i,1+length(B_aug(1,:))*(j-1):length(B_aug(1,:))*j)=A_aug^(i-j)*B_aug;
           end
       end
       Adc(1+length(A_aug(:,1))*(i-1):length(A_aug(:,1))*i,1:length(A_aug(1,:)))=A_aug^(i);
       
       %% Constraints
       y_max_global(length(y_max)*(i-1)+1:length(y_max)*i)=y_max;
       y_min_global(length(y_min)*(i-1)+1:length(y_min)*i)=y_min;
       
    end
    Hdb=Cdb'*Qdb*Cdb+Rdb;
    Fdbt=[Adc'*Qdb*Cdb;-Tdb*Cdb];
end
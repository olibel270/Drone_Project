%initialize to zero
time_vector = [0:time_interval:60];
desired_pos = [zeros(4,size(time_vector,2));time_vector];

%set desired trajectory
    %hover
    desired_pos(3,:) = -3;

    %x=10,y=0
    desired_pos(1,desired_pos(5,:)>=5 & desired_pos(5,:)<10) = 10;
    desired_pos(2,desired_pos(5,:)>=5 & desired_pos(5,:)<10) = 0;
    
    %x=0,y=10
    desired_pos(1,desired_pos(5,:)>=10 & desired_pos(5,:)<15) = 0;
    desired_pos(2,desired_pos(5,:)>=10 & desired_pos(5,:)<15) = 10;
    
    %x=-10,y=0
    desired_pos(1,desired_pos(5,:)>=15 & desired_pos(5,:)<20) = -10;
    desired_pos(2,desired_pos(5,:)>=15 & desired_pos(5,:)<20) = 0;
    
    %x=0,y=-10
    desired_pos(1,desired_pos(5,:)>=20 & desired_pos(5,:)<25) = 0;
    desired_pos(2,desired_pos(5,:)>=20 & desired_pos(5,:)<25) = -10;
    
    %x=10,y=0
    desired_pos(1,desired_pos(5,:)>=25 & desired_pos(5,:)<30) = 10;
    desired_pos(2,desired_pos(5,:)>=25 & desired_pos(5,:)<30) = 0;
    
    %x=0,y=0
    desired_pos(1,desired_pos(5,:)>=30) = 0;
    desired_pos(2,desired_pos(5,:)>=30) = 0;
    
    
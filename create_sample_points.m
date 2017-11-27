function Points= create_sample_points(V_max,t_trans,d_thresh,n_max)
% n_max=1000;
% global V_max t_trans
% for i=1:n_max
%     Points(i).Pa1= 60*rand(2,1) -30;
%     Points(i).Pa2= 60*rand(2,1) -30;
%     theta1= rand(1,1)*2*pi;
%     d1=20*rand(1,1) + 10;
%     Points(i).Pb1 = Points(i).Pa1 + [d1*cos(theta1) ; d1*sin(theta1) ];
%     
%     theta2= rand(1,1)*2*pi;
%     d2=20*rand(1,1) + 10;
%     Points(i).Pb2 = Points(i).Pa2 + [d1*cos(theta2); d1*sin(theta2)];
% end
map_x=30;
map_y=30;
trans_time=t_trans;
vmax= (V_max-2)/(2^.5);
d_min=d_thresh;
i=0;

while i<=n_max
    i=i+1;
    Points(i).Pa1(1,1)= (2*map_x)*rand(1,1) -map_x;
    Points(i).Pa1(2,1)= (2*map_y)*rand(1,1) -map_y;
    Points(i).Va(1,1)= 2*vmax*rand(1,1) -vmax;
    Points(i).Va(2,1)= 2*vmax*rand(1,1) -vmax;
    Points(i).Pa2 = Points(i).Pa1 + trans_time.*Points(i).Va;
    
    t_col = (trans_time-.5)*rand(1,1) + .5;
    Pa_col = Points(i).Pa1 + Points(i).Va.*t_col;
    
    dx= 2*d_min*rand(1,1) - d_min;
    dy= 2*d_min*rand(1,1) - d_min;
    Pb_col = Pa_col + [dx;dy];
    Points(i).Vb(1,1)= 2*vmax*rand(1,1) -vmax;
    Points(i).Vb(2,1)= 2*vmax*rand(1,1) -vmax;
    
    Points(i).Pb1= Pb_col - t_col.*Points(i).Vb;
    Points(i).Pb2= Points(i).Pb1 + trans_time.*Points(i).Vb;
    
    if norm(Points(i).Pa1 - Points(i).Pb1)<10 || norm(Points(i).Pa2 - Points(i).Pb2)<10
        i=i-1;
    end
        
    
    
%     plot([Points(i).Pa1(1,1),Points(i).Pa2(1,1)],[Points(i).Pa1(2,1),Points(i).Pa2(2,1)])
%     hold on
%      plot([Points(i).Pb1(1,1),Points(i).Pb2(1,1)],[Points(i).Pb1(2,1),Points(i).Pb2(2,1)])
%     
%     drawnow;
%     hold off
%     
end

end
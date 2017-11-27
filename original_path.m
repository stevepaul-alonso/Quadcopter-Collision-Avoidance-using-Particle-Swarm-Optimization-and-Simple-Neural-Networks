function[min_sep]= original_path( V1, V2, Pa_i, Pb_i, Pa_f, Pb_f,t_trans)
t=0;
dt=0.05;
Pa=Pa_i;
Pb=Pb_i;
mi_x=min([Pa_i(1,1) Pb_i(1,1) Pa_f(1,1) Pb_f(1,1)])-10;
mi_y=min([Pa_i(2,1) Pb_i(2,1) Pa_f(2,1) Pb_f(2,1)])-10;
ma_x=max([Pa_i(1,1) Pb_i(1,1) Pa_f(1,1) Pb_f(1,1)])+10;
ma_y=max([Pa_i(2,1) Pb_i(2,1) Pa_f(2,1) Pb_f(2,1)])+10;
sl=0;
while t<=t_trans
    sl=sl+1;
    t=t+dt;
    Pa=Pa+V1.*dt;
    Pb=Pb+V2.*dt;
    sep(sl,1)=norm(Pa-Pb);
    plot(Pa_i(1,1),Pa_i(2,1),'*')
    hold on
    plot(Pb_i(1,1),Pb_i(2,1),'*')
    hold on
    plot(Pa_f(1,1),Pa_f(2,1),'*')
    hold on
    plot(Pb_f(1,1),Pb_f(2,1),'*')
    hold on
    plot(Pa(1,1),Pa(2,1),'*')
    hold on
    plot(Pb(1,1),Pb(2,1),'*')
    xlim([mi_x ma_x])
    ylim([mi_y ma_y])
    drawnow;
    hold off
end
min_sep=min(sep);
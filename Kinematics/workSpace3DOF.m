close all;
clearvars;

stepsTheta1 = 200;
stepsTheta2 = 200;
stepsTheta3 = 200*2;

step_per_deg1=(2*pi)/stepsTheta1;
step_per_deg2=(2*pi)/stepsTheta2;
step_per_deg3=(2*pi)/stepsTheta3;
figure 
hold on
i = 1;
P = zeros(3,stepsTheta1*stepsTheta2*stepsTheta3);
for q1 = -pi/3:step_per_deg1:4*pi/3
    for q2 = 0:step_per_deg2:pi 
        for q3= -pi/2:step_per_deg3:pi/2
            [px,py,pz] = ForwardKinematics(q1,q2,q3);
            P(:,i) = [px;py;pz];
            i = i + 1;
        end
    end
end

plot3(P(1,:),P(2,:),P(3,:),'.r')
xlabel('X')
ylabel('Y')
zlabel('Z')

axis equal
view(3)
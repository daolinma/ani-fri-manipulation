
size_q = size(q);

figure(100)
close(figure,100)
figure(100)

axis equal
plot(q(1,1:5:size_q(2)), q(2,1:5:size_q(2)),'r')
hold on 

for idx=1:5:1600
    A = [q(1,idx), q(2,idx)] + [ a/2, b/2] * [cos(q(3,idx)), sin(q(3,idx)); -sin(q(3,idx)),cos(q(3,idx))];
    B = [q(1,idx), q(2,idx)] + [-a/2, b/2] * [cos(q(3,idx)), sin(q(3,idx)); -sin(q(3,idx)),cos(q(3,idx))];
    C = [q(1,idx), q(2,idx)] + [-a/2,-b/2] * [cos(q(3,idx)), sin(q(3,idx)); -sin(q(3,idx)),cos(q(3,idx))];
    D = [q(1,idx), q(2,idx)] + [ a/2,-b/2] * [cos(q(3,idx)), sin(q(3,idx)); -sin(q(3,idx)),cos(q(3,idx))];
    plot([A(1),B(1)],[A(2),B(2)])
    plot([C(1),B(1)],[C(2),B(2)])
    plot([C(1),D(1)],[C(2),D(2)])
    plot([D(1),A(1)],[D(2),A(2)])
    fd=1 
    axis equal
%     line(q(1,idx)- a/2* cos(q(3,idx)),q(2,idx)+b/2*sin(q(3,idx)))
%     line(q(1,idx)+ a/2*cos(q(3,idx)),q(2,idx)-b/2*sin(q(3,idx)))
%     line(q(1,idx)- a/2*cos(q(3,idx)),q(2,idx)-b/2*sin(q(3,idx)))
end

hold off


figure(101)
% axis equal
plot(q(1,1:2:size_q(2)), q(2,1:2:size_q(2)),'r')
figure(102)
% axis equal
plot(q(3,1:2:size_q(2)),'r')
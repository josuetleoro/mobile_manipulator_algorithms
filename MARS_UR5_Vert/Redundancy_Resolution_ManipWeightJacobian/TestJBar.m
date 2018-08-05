clear all

q=[0;0;0;0;0;0;0;0;0.0;0];
eta=[0;0;0;0;0;0;0;0;0.1];
JBar=evaluateJBar(q(3),q(5),q(6),q(7),q(8),q(9));

x=JBar*eta
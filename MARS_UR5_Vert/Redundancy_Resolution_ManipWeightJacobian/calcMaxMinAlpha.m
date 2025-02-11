function [maxAlpha, minAlpha]=calcMaxMinAlpha(dqp,dqh,dq_limit)
n=size(dqp);
kmax = Inf;
kmin = -Inf;
for k=1:n
   kmax_aux = max((dq_limit(k)-dqp(k))/dqh(k),(-dq_limit(k)-dqp(k))/dqh(k));
   kmin_aux = min((dq_limit(k)-dqp(k))/dqh(k),(-dq_limit(k)-dqp(k))/dqh(k));
   kmax = min(kmax_aux,kmax);
   kmin = max(kmin_aux,kmin);
end
maxAlpha = kmax;
minAlpha = kmin;
end
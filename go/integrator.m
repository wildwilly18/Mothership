%Integrator Error.
err_vec = 1;

time = 1:0.1:10;
int = 0;
ints = zeros(length(time),1);
for ii = 1: length(time)
    if ii == 1
        int = 1;
    else
        ints(ii) = ints(ii-1, 1) + 0.5*(err_vec + err_vec);
    end

end

plot(time, ints)
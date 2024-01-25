function out = ezrand(rmin,rmax)
    out = rand(size(rmin))*(rmax-rmin)+rmin;
end


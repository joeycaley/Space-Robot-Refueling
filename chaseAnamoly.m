function fc = chaseAnam(fg,minGap)

if fg < 2*pi-minGap
    fc = (2*pi-2*minGap)*rand + fg + minGap;
    if fc > 2*pi
        fc = rem(fc,2*pi);
    end
else
    fc = fg - minGap - (2*pi-2*minGap)*rand;
    if fc < 0
        fc = fc + 2*pi;
        if fc < 0 
            fc = fc + 2*pi;
        end
    end
end


end
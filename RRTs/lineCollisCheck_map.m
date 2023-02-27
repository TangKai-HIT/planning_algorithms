 function connected = lineCollisCheck_map( p_start, p_end, map  )
%LINECOLLISCHECK_MAP Checks a line for collision (modified from CMU toolbox)
%   p_start: start coordinates 1x2
%   p_end: end coordinates 1x2
%   map: map struct
%   connected: 0 if free, 1 if not
c = value_line_map( p_start, p_end, map );

if (any(~c))
    connected = 1;
else
    connected = 0;
end

end


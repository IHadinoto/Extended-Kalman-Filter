% Function for parsing the binary scan; extracting ranges and intensities of reflections.
function [r,I]=GetRangeAndIntensityFromRawScan_z5161724_Hadinoto_Ian(scan)
    r = 0.01*single(bitand(scan,8191));
    I= bitshift(scan,-13);      
    % bits [0:12] are range, bits [13:15] intensity
end
function [pos_x, pos_y] = findPosition(mask)
    
% find the x position and y position for current image
% and estimate the camera center relative to the panorama
% x coordinate is defined as the 1/2 value of each image’s horizontal coordinate
% y position defined as the 1/3 distance of y coordinates from bottom to top

    flag = 0;
    for i = 1 : size(mask, 2)
        for j = 1 : size(mask, 1)
           if mask(j, i) == 1
               pos_start = i;
               flag = 1;
               break;
           end                     
        end
        if flag == 1
            break
        end
    end
    
    flag = 0;
    for i = size(mask, 2) : -1 : 1
        for j = 1 : size(mask, 1)
           if mask(j, i) == 1
               pos_end = i;
               flag = 1;
               break;
           end                     
        end
        if flag == 1
            break;
        end
    end
    
    pos_x = pos_start + floor((pos_end - pos_start)/2);
    
    flag = 0;
    for i = size(mask, 1) : -1 : 1
        for j = 1 : size(mask, 2)
           if mask(i, j) == 1
               pos_start = i;
               flag = 1;
               break;
           end                     
        end
        if flag == 1
            break;
        end
    end
    
    flag = 0;
    for i = 1 : size(mask, 1)
        for j = 1 : size(mask, 2)
           if mask(i, j) == 1
               pos_end = i;
               flag = 1;
               break;
           end                     
        end
        if flag == 1
            break;
        end
    end
    
    pos_y = pos_start - floor((pos_start - pos_end)/3);

end
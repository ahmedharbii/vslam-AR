%% function to decide whether to change ref image or not
function [change] = change_ref_or_not(norm_x, tracking_param)
change = false;
if norm_x < tracking_param.changereference_thresh
    change = true
end
return
end


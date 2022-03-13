%% function to decide whether to change ref image or not
function [change] = change_ref_or_not(data, tracking_param)
change = false;
if data.norm_residues <= tracking_param.changereference_thresh
    change = true
end
return
end


function orientation_euler = getEulerOrientation(orientation_quart)
    orientation_euler = ...
        quatern2euler(quaternConj(orientation_quart));
end
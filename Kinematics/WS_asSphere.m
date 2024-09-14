function [] = WS_asSphere()
global x y z x_blur y_blur z_blur
surf(x-5, y, z, 'FaceColor', 'blue', 'EdgeColor', 'non','FaceAlpha', 0.1);
surf(x_blur, y_blur, z_blur, 'FaceColor', 'blue', 'EdgeColor', 'non','FaceAlpha', 0.1);
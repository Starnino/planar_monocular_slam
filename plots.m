figure(1)
hold on;
grid;

subplot(2,2,1);
plot3(XL_true(1,:),XL_true(2,:),XL_true(3,:),'b*',"linewidth",2);
hold on;
plot3(XL_guess(1,:),XL_guess(2,:),XL_guess(3,:),'ro',"linewidth",2);
title("Landmark Initial Guess");
legend("True", "Guess");grid;

subplot(2,2,2);
plot3(XL_true(1,:),XL_true(2,:),XL_true(3,:),'b*',"linewidth",2);
hold on;
plot3(XL(1,:),XL(2,:),XL(3,:),'ro',"linewidth",2);
title("Landmark After Optimization");
legend("True", "Guess");grid;

subplot(2,2,3);
plot3(XR_true(:,1),XR_true(:,2),XR_true(:,3),'b*',"linewidth",2);
hold on;
plot3(XR_guess(:,1),XR_guess(:,2),XR_guess(:,3),'ro',"linewidth",2);
title("Poses Initial Guess");
legend("True", "Guess");grid;

subplot(2,2,4);
plot3(XR_true(:,1),XR_true(:,2),XR_true(:,3),'b*',"linewidth",2);
hold on;
plot3(XR(:,1),XR(:,2),XR(:,3),'ro',"linewidth",2);
title("Poses After Optimization");
legend("True", "Guess"); grid;

figure(2);
hold on;
grid;
title("chi evolution");

subplot(3,2,1);
plot(chi_stats(1,:), 'r-', "linewidth", 2);
legend("Chi Poses"); grid; xlabel("iterations");
subplot(3,2,2);
plot(num_inliers(1,:), 'b-', "linewidth", 2);
legend("#inliers"); grid; xlabel("iterations");

subplot(3,2,5);
plot(chi_stats(2,:), 'r-', "linewidth", 2);
legend("Chi Proj"); grid; xlabel("iterations");
subplot(3,2,6);
plot(num_inliers(2,:), 'b-', "linewidth", 2);
legend("#inliers");grid; xlabel("iterations");

figure(3);
title("H matrix");
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_)) = 0;               # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
hold off;
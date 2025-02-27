video = VideoWriter('state1to3demo.avi'); % You can change the file format if needed
video.FrameRate = 1/dt/3; % Set the frame rate to match 0.05 seconds per frame
open(video); % Open the video file for writing
results_noised{1}.
for i=results_noised{1}.t
    show_fig(Model_consts,State_variables,i);
    
    text(COM_x, -2*Model_consts('r') - 0.5, sprintf('COMv: %.3f', COMv_x), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    text(COM_x, -2*Model_consts('r') - 0.7, sprintf('state: %.3f', controller_state), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    frame = getframe(gcf);
    writeVideo(video, frame);
end
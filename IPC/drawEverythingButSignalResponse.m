function CLTF = drawEverythingButSignalResponse(TF)
    % Plots bode diagram, root locus, nyquist diagram and step response of
    % the given transfer function TF
    % as well as the time response to the reference signal RS 
    % (in the time domain) in the timestamp given by T
    subplot(2, 2, 1);
    margin(TF);
    grid on;
    
    subplot(2, 2, 2);
    rlocus(TF);
    grid on;

    subplot(2, 2, 3);
    nyquist(TF);
    grid on;
    
    CLTF = minreal(TF / (1 + TF));

    subplot(2, 2, 4);
    step(CLTF, 100);
    grid on;
end
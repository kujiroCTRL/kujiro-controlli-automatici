function CLTF = drawEverything(TF, T, RS)
    % Plots bode diagram, root locus, nyquist diagram and step response of
    % the given transfer function TF
    % as well as the time response to the reference signal RS 
    % (in the time domain) in the timestamp given by T
    figure(1);
    
     CLTF = drawEverythingButSignalResponse(TF);

    figure(2);
    lsim(CLTF, T, RS);
    grid on;
end
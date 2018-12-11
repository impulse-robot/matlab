function [fitresult, gof] = spline_fit(input_angles, end_effector, visualize)
%SPLINE_FIT(INPUT_ANGLES,END_EFFECTOR)
%  Create a fit.
%
%  Data for 'spline_fit' fit:
%      X Input : input_angles
%      Y Output: end_effector
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%% Fit: 'spline_fit'.
[xData, yData] = prepareCurveData( input_angles, end_effector );

% Set up fittype and options.
ft = fittype( 'smoothingspline' );

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, 'SmoothingParam', 1.0 );

% Plot fit with data.
if (visualize)
    figure( 'Name', 'Forward Kinematics Fit' );
    h = plot( fitresult, xData, yData );
    legend( h, 'End Effector vs. Input Angles', 'Fitted spline', ...
        'Location', 'NorthEast' );
    % Label axes
    xlabel('\theta [rad]')
    ylabel('Foot position y [m]')
    grid on
    hold on
end



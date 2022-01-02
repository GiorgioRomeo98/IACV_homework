function plot_lines(lines)
%PLOT_LINES plot lines over the image
%
% input
% lines: vector of structs where each struct is composed by two points
% belonging to the line (point1, point2) plus two other (optional) parameters theta and
% rho as the ones returns by the Hough Transform (hough --> houghpeaks --> houghlines)


%plot the lines
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',0.5,'Color','red');

        % Plot beginnings and ends of lines
        % plot(xy(1,1),xy(1,2),'x','LineWidth',0.05,'Color','red');
        % plot(xy(2,1),xy(2,2),'x','LineWidth',0.05,'Color','red');
        text(xy(1,1),xy(1,2), num2str(k), 'Color', 'green', 'FontSize',8)
    end
end


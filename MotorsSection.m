% [x, y] = MotorsSection(obj, action, x, y)
%
% Section that takes care of controlling the stepper motors.
%
%
% PARAMETERS:
% -----------
%
% obj      Default object argument.
%
% action   One of:
%            'init'      To initialise the section and set up the GUI
%                        for it;
%
%            'reinit'    Delete all of this section's GUIs and data,
%                        and reinit, at the same position on the same
%                        figure as the original section GUI was placed.
%
%            Several other actions are available (see code of this file).
%
% x, y     Relevant to action = 'init'; they indicate the initial
%          position to place the GUI at, in the current figure window
%
% RETURNS:
% --------
%
% [x, y]   When action == 'init', returns x and y, pixel positions on
%          the current figure, updated after placing of this section's GUI.
%
% x        When action = 'get_next_side', x will be either 'l' for
%          left or 'r' for right.
%

function [x, y] = MotorsSection(obj, action, x, y)

GetSoloFunctionArgs;
global radius
global angleToPostGoPosition;
global angle_startTOantGO;
global angle_startTOantGAP;
global angle_startTOantNOGO;
global Solo_Try_Catch_Flag
global motors_properties;
global motors;

switch action

    case 'init',   % ------------ CASE INIT ----------------

        if strcmp(motors_properties.type,'@FakeZaberAMCB2')
            motors = FakeZaberAMCB2;
        else
            disp(['Real Motor!!!']);
            motors = ZaberAMCB2(motors_properties.port);
        end

        %         disp('trying to open motors');
        serial_open(motors);
        disp('motors are open');
        %         disp(motors);

        % Save the figure and the position in the figure where we are
        % going to start adding GUI elements:
        SoloParamHandle(obj, 'my_gui_info', 'value', [x y gcf]); next_row(y,1.5);
        %        SoloParamHandle(obj, 'motor_num', 'value', 0);

        %added by ZG 10/1/11
        SoloParamHandle(obj, 'motor_num', 'value', 1);
        SoloParamHandle(obj, 'lateral_motor_num', 'value', 2);

        % List of pole positions
        SoloParamHandle(obj, 'previous_pole_positions', 'value', []);
        SoloParamHandle(obj, 'previous_pole_positions_lat', 'value', []);

        % Set limits in microsteps for actuator. Range of actuator is greater than range of
        % our Del-Tron sliders, so must limit to prevent damage.  This limit is also coded into Zaber
        % TCD1000 firmware, but exists here to keep GUI in range. If a command outside this range (0-value)
        % motor driver gives error and no movement is made.
        SoloParamHandle(obj, 'motor_max_position', 'value', 180000);
        SoloParamHandle(obj, 'trial_ready_times', 'value', 0);

        MenuParam(obj, 'motor_show', {'view', 'hide'}, 'hide', x, y, 'label', 'Motor Control', 'TooltipString', 'Control motors');
        set_callback(motor_show, {mfilename,'hide_show'});

        next_row(y);
        SubheaderParam(obj, 'sectiontitle', 'Motor Control', x, y);

        parentfig_x = x; parentfig_y = y;

        % List of pole radii for on axes mode
        SoloParamHandle(obj, 'previous_pole_radii', 'value', []);

        % List of pole angles for on axes mode
        SoloParamHandle(obj, 'previous_pole_angles', 'value', []);

        % ---  Make new window for motor configuration
        SoloParamHandle(obj, 'motorfig', 'saveable', 0);
        motorfig.value = figure('Position', [3 500 400 400], 'Menubar', 'none',...
            'Toolbar', 'none','Name','Motor Control','NumberTitle','off');

        x = 1; y = 1;

        %       PushButtonParam(obj, 'serial_open', x, y, 'label', 'Open serial port');
        %       set_callback(serial_open, {mfilename, 'serial_open'});
        %       next_row(y);

        PushButtonParam(obj, 'serial_reset', x, y, 'label', 'Reset serial port connection');
        set_callback(serial_reset, {mfilename, 'serial_reset'});
        next_row(y);

        %         PushButtonParam(obj, 'reset_motors_firmware', x, y, 'label', 'Reset Zaber firmware parameters',...
        %             'TooltipString','Target acceleration, target speed, and microsteps/step');
        %         set_callback(reset_motors_firmware, {mfilename, 'reset_motors_firmware'});
        %         next_row(y);


        PushButtonParam(obj, 'motors_home', x, y, 'label', 'Home motor');
        set_callback(motors_home, {mfilename, 'motors_home'});
        next_row(y);

        PushButtonParam(obj, 'motors_stop', x, y, 'label', 'Stop motor');
        set_callback(motors_stop, {mfilename, 'motors_stop'});
        next_row(y);

        PushButtonParam(obj, 'motors_reset', x, y, 'label', 'Reset motor');
        set_callback(motors_reset, {mfilename, 'motors_reset'});
        next_row(y, 2);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%adding joystick gui -PSM
        PushButtonParam(obj, 'move_down', x, y, 'label', 'DOWN');
        set_callback(move_down, {mfilename, 'move_down'});
        next_row(y);

        PushButtonParam(obj, 'move_right', x, y, 'label', 'RIGHT');
        set_callback(move_right, {mfilename, 'move_right'});
        next_row(y);

        PushButtonParam(obj, 'move_left', x, y, 'label', 'LEFT');
        set_callback(move_left, {mfilename, 'move_left'});
        next_row(y);

        PushButtonParam(obj, 'move_up', x, y, 'label', 'UP');
        set_callback(move_up, {mfilename, 'move_up'});
        next_row(y);

        NumeditParam(obj, 'joystick_increment', 8000, x, y, 'label', ...
            'JoystickIncrement','TooltipString','set joystick step size');
        next_row(y,2);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        NumeditParam(obj, 'nogo_degrees', 20, x, y, 'label', ...
            'NOGO_degrees','TooltipString','set nogo angle size in degrees');
        next_row(y);

        NumeditParam(obj, 'gap_degrees', 15, x, y, 'label', ...
            'GAP_degrees','TooltipString','set go angle size in degrees');
        next_row(y);

        NumeditParam(obj, 'go_degrees', 20, x, y, 'label', ...
            'GO_degrees','TooltipString','set go angle size in degrees');
        next_row(y);

        SubheaderParam(obj, 'title', 'Arc_mode_set', x, y);
        next_row(y);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        MenuParam(obj, 'sample_mode', {'off' 'threshold' 'adaptive'}, 'adaptive', x, y,...
            'TooltipString','threshold- reach over sample space redirected || adaptive - threshold withbias towards un sampled.');
        next_row(y);

        MenuParam(obj, 'position_mode', {'arc' 'normal' 'on_axes' 'grid_off_axes'}, 'normal', x, y,...
            'TooltipString','Arc- set by degrees and post go position');
        next_row(y);

        PushButtonParam(obj, 'show_go_arc', x, y, 'label', 'show_go_arc');
        set_callback(show_go_arc, {mfilename, 'show_go_arc'});
        next_row(y);

        PushButtonParam(obj, 'show_gap_arc', x, y, 'label', 'show_gap_arc');
        set_callback(show_gap_arc, {mfilename, 'show_gap_arc'});
        next_row(y);

        PushButtonParam(obj, 'show_nogo_arc', x, y, 'label', 'show_nogo_arc');
        set_callback(show_nogo_arc, {mfilename, 'show_nogo_arc'});
        next_row(y);
        %%%%these need to be implemented by making an array of positions
        %%%%and then the motor position has go through the positions with a
        %%%%very small pause to show user where the pole will be.-PSM
        %%%%%%%%%%%%%%%%%%%%%%%%%%%adding joystick gui -PSM


        next_column(x); y = 1;

        PushButtonParam(obj, 'read_positions', x, y, 'label', 'Read position');
        set_callback(read_positions, {mfilename, 'read_positions'});

        next_row(y);
        NumeditParam(obj, 'motor_position', 0, x, y, 'label', ...
            'Motor position','TooltipString','Absolute position in microsteps of motor.');
        set_callback(motor_position, {mfilename, 'motor_position'});

        next_row(y);
        SubheaderParam(obj, 'title', 'Read/set position', x, y);




        %--------------- extreme positions for the multi-pole task --------------------------------
        next_row(y);
        NumeditParam(obj, 'no_pole_position_ant', 180000, x, y, 'label', ...
            '"No" ant position','TooltipString','Far no trial position in microsteps.');

        next_row(y);
        NumeditParam(obj, 'no_pole_position_pos', 100001, x, y, 'label', ...
            '"No" pos position','TooltipString','Near no trial position in microsteps.');

        next_row(y);
        NumeditParam(obj, 'yes_pole_position_ant', 100000, x, y, 'label', ...
            '"Yes" ant position','TooltipString','Far yes trial position in microsteps.');

        next_row(y);
        NumeditParam(obj, 'yes_pole_position_pos', 20000, x, y, 'label', ...
            '"Yes" pos position','TooltipString','Near yes trial position in microsteps.');
        %
        %         next_row(y);
        %         NumeditParam(obj, 'num_of_pole_position', 5, x, y, 'label', ...
        %             'Pole positions','TooltipString','Number of Yes/No pole position');
        %
        %         % switch between 2 pole task and multi-pole
        %         next_row(y);
        %         ToggleParam(obj, 'multi_go_position', 0, x, y, 'label', 'Multi Go Positions',...
        %             'TooltipString', 'Multiple pole position will be used.');
        %-----------------------------------------------------------



        next_row(y);
        NumeditParam(obj, 'motor_move_time', 2, x, y, 'label', ...
            'motor move time','TooltipString','set up time for motor to move.');

        next_row(y)
        PushButtonParam(obj, 'read_lateral_positions', x, y, 'label', 'Read lateral position');
        set_callback(read_lateral_positions, {mfilename, 'read_lateral_positions'});

        next_row(y);
        NumeditParam(obj, 'lateral_motor_position', 50000, x, y, 'label', ...
            'lateral_motor_position','TooltipString','Absolute position in microsteps of motor.');
        set_callback(lateral_motor_position, {mfilename, 'lateral_motor_position'});

        next_row(y);
        SubheaderParam(obj, 'title', 'Trial position', x, y);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        next_row(y);
        NumeditParam(obj, 'post_go_lat', 0, x, y, 'label', ...
            'post_go_lat','TooltipString','posterior go position for lateral motor(arc mode)');
        set_callback(post_go_lat, {mfilename, 'post_go_lat'});
        next_row(y);
        NumeditParam(obj, 'post_go', 0, x, y, 'label', ...
            'post_go','TooltipString','posterior go position for main motor(arc mode)');
        set_callback(post_go, {mfilename, 'post_go'});
        next_row(y);
        PushButtonParam(obj, 'set_both_post_go', x, y, 'label', ...
            'set_both_post_go','TooltipString',...
            'set currents motor position as posterior go position for main and lateral motors(arc mode)');
        set_callback(set_both_post_go, {mfilename, 'set_both_post_go'});
        next_row(y);
        NumeditParam(obj, 'center_lat', 0, x, y, 'label', ...
            'center_lat','TooltipString','center_lateral (y value) for the arc');
        set_callback(center_lat, {mfilename, 'center_lat'});
        next_row(y);
        NumeditParam(obj, 'center_main', 0, x, y, 'label', ...
            'center_main','TooltipString','center_main (x value) for the arc');
        set_callback(center_main, {mfilename, 'center_main'});
        next_row(y);
        PushButtonParam(obj, 'set_center', x, y, 'label', ...
            'set_center','TooltipString',...
            'set_center of arc(arc mode)');
        set_callback(set_center, {mfilename, 'set_center'});
        next_row(y);
        NumeditParam(obj, 'normLatMotPositionSET', 50000, x, y, 'label', ...
            'normLatMotPositionSET','TooltipString','set by user. set to value of lat position for linear ''normal''task for calculating radial distance rage');
        set_callback(normLatMotPositionSET, {mfilename, 'normLatMotPositionSET'});
        next_row(y);
        SubheaderParam(obj, 'title', 'Arc_mode_set', x, y);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        MotorsSection(obj,'hide_show');
        MotorsSection(obj,'read_positions');
        MotorsSection(obj,'read_lateral_positions');


        % Give read-only access to SideSection.m
        SoloFunctionAddVars('SidesSection', 'ro_args', 'position_mode');

        x = parentfig_x; y = parentfig_y;
        set(0,'CurrentFigure',value(myfig));
        return;

    case 'move_next_side', % --------- CASE MOVE_NEXT_SIDE -----

        next_side = SidesSection(obj,'get_next_side');
        next_trial_type = SidesSection(obj,'get_next_trial');

        %---------ON_AXES---------
        if strcmp(value(position_mode), 'on_axes')
            %probe just the axes








            %theta's and radial distances for linear task
            linearModeRadiusPostGo = sqrt(abs((yes_pole_position_pos(:)-center_main(:))^2) + abs((normLatMotPositionSET(:)-center_lat(:))^2));
            linearModeAngleToPostGoPosition = asind((center_main(:)-yes_pole_position_pos(:)) / linearModeRadiusPostGo);

            linearModeRadiusPostGap = sqrt(abs((yes_pole_position_ant(:)-center_main(:))^2) + abs((normLatMotPositionSET(:)-center_lat(:))^2));
            linearModeAngle_startTOantGO = asind((center_main(:)-yes_pole_position_ant(:)) / linearModeRadiusPostGap);

            linearModeRadiusPostNoGo = sqrt(abs((no_pole_position_pos(:)-center_main(:))^2) + abs((normLatMotPositionSET(:)-center_lat(:))^2));
            linearModeAngle_startTOantGAP = asind((center_main(:)-no_pole_position_pos(:)) / linearModeRadiusPostNoGo);

            linearModeRadiusAntNoGo = sqrt(abs((no_pole_position_ant(:)-center_main(:))^2) + abs((normLatMotPositionSET(:)-center_lat(:))^2));
            linearModeAngle_startTOantNOGO = asind((center_main(:)-no_pole_position_ant(:)) / linearModeRadiusAntNoGo);

            radArray = [linearModeRadiusPostGo, linearModeRadiusPostGap, linearModeRadiusPostNoGo, linearModeRadiusAntNoGo];%4 radial distances starting from go posterior
            angArray = [linearModeAngleToPostGoPosition, linearModeAngle_startTOantGO, linearModeAngle_startTOantGAP, linearModeAngle_startTOantNOGO];%4 angles starting from go posterior

            indexMinRadialDistance = find(angArray<0);%find neg values to find which (if any) range has the absolute min radial distance(i.e. no X component)
            if  ~isempty(indexMinRadialDistance)
                indexMinRadialDistance = indexMinRadialDistance(end);
            end



            decisionBoundaryRadialDistance = (linearModeRadiusPostGap + linearModeRadiusPostNoGo)/2;


            %in case you need to rotate the start angle. set to 0 otherwise. this was
            %not implemented so be careful if you use this adjustment value and make
            %sure it is incorporated into everything properly (did not incorporate into
            %radial distance task or any grid task) -PSM
            AddedAngle = 0;

            radius = decisionBoundaryRadialDistance;

            go_degrees = abs(angArray(2)-angArray(1));
            gap_degrees = abs(angArray(3)-angArray(2));
            nogo_degrees = abs(angArray(4)-angArray(3));

            % ArcLengthGO =   2*pi*radius*go_degrees/360;
            % ArcLengthGAP =  2*pi*radius*gap_degrees/360;
            % ArcLengthNOGO = 2*pi*radius*nogo_degrees/360;

            %in degrees
            angleToPostGoPosition = angArray(1);
            angle_startTOantGO = angArray(2);
            angle_startTOantGAP = angArray(3);
            angle_startTOantNOGO = angArray(4);

            decisionBoundaryAngle = angle_startTOantGO + abs((angle_startTOantGAP - angle_startTOantGO)/2);
            %for no gap this value will equal angle_startTOantGO


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%THE F'IN RADIAL%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if next_trial_type == 'r'%radial
                %%%%%%%%%%%%%%%start code for implementing warning%%%%%%%%%%%%%%%%
                %because a negative angle will cause potential overlap between go and nogo
                %radial distances calculated from linear task, I will design the program to
                %alert if this is happening.

                nearRadialDistance = sqrt(abs((normLatMotPositionSET(:)-center_lat(:))^2));
                if indexMinRadialDistance == 1 %min in the go range
                    startGoRange = nearRadialDistance;
                    endGoRange = max(radArray(1), radArray(2));
                    goRange = abs(startGoRange-endGoRange);

                    startNoGoRange = radArray(3);
                    endNoGoRange = radArray(4);
                    noGoRange =abs(startNoGoRange-endNoGoRange);
                    if endGoRange>startNoGoRange
                        warning('you have overlapping go and nogo radial distance ranges')
                    end

                elseif indexMinRadialDistance == 2 %min in the gap range
                    %when min range is in the gap area, the min distance
                    %doesn't change anything.
                    startGoRange = radArray(1);
                    endGoRange = radArray(2);
                    goRange = abs(startGoRange-endGoRange);

                    startNoGoRange = radArray(3);
                    endNoGoRange = radArray(4);
                    noGoRange =abs(startNoGoRange-endNoGoRange);

                elseif indexMinRadialDistance == 3 %min in the nogo range

                    startGoRange = radArray(1);
                    endGoRange = radArray(2);
                    goRange = abs(startGoRange-endGoRange);

                    startNoGoRange = nearRadialDistance;
                    endNoGoRange = max(radArray(4),radArray(3));
                    noGoRange =abs(startNoGoRange-endNoGoRange);
                    if endGoRange>startNoGoRange
                        '*************************'
                        '*************************'
                        '*************************'
                        warning('you have overlapping go and nogo radial distance ranges')
                        '*************************'
                        '*************************'
                        '*************************'
                    end

                elseif indexMinRadialDistance == 4 %min outside of all ranges and nogo anterior is posterior to midpoint
                    warning('the motors may be arranged in a way that is incompatable with the program. positive X should move posterior to mouse, and positive Y should move towards mouse face.')
                    nearRadialDistance = linearModeRadiusPostGo;
                else             %min outside of all ranges and nogo anterior is posterior to midpoint
                    nearRadialDistance = linearModeRadiusPostGo;

                    startGoRange = radArray(1);
                    endGoRange = radArray(2);
                    goRange = abs(startGoRange-endGoRange);

                    startNoGoRange = nearRadialDistance;
                    endNoGoRange = max(radArray(4),radArray(3));
                    noGoRange =abs(startNoGoRange-endNoGoRange);
                end

                if goRange<0 || noGoRange<0
                    while limitedGoRange<0 || limitedNoGoRange<0
                        'ACTION REQUIRED'
                        '*************************'
                        '*************************'
                        warning('either NOGO or GO radial range is negative. can''t use these positions to probe radial ranges')
                        '*************************'
                        '*************************'
                        'PAUSE 10 SEC PLEASE CHANGE'
                        pause(10)
                    end
                elseif goRange==0 || noGoRange==0
                    ''
                    ''
                    warning('either NoGo or Go range is 0, ONLY PROBING ONE POSITION')
                    ''
                    ''
                else
                    startGoRange
                    endGoRange
                    goRange
                    startNoGoRange
                    endNoGoRange
                    noGoRange
                end
                %%%%%%%%%%%%%%%%%end code for implementing warning%%%%%%%%%%%%%%%%

                if next_side == 'r'
                    next_pole_radius = round(startGoRange + rand*(goRange));
                    startRange = startGoRange;
                    endRange = endGoRange;
                    range = goRange;
                elseif next_side == 'l'
                    next_pole_radius = round(startNoGoRange + rand*(noGoRange));
                    startRange = startNoGoRange;
                    endRange = endNoGoRange;
                    range = noGoRange;
                else
                    error('un-recognized type for next_side');
                end
                next_pole_XXXX = next_pole_radius;
                previous_pole_XXXX = previous_pole_radii;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%THE F'IN ARC%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elseif next_trial_type == 'a'
                if  next_side == 'r'
                    startRange = angArray(1);
                    endRange = angArray(2);
                    range = go_degrees;
                    next_pole_angle = endRange + (startRange-endRange)*rand(1);
                elseif next_side == 'l'
                    startRange = angArray(3);
                    endRange = angArray(4);
                    range = nogo_degrees;
                    next_pole_angle = endRange + (startRange-endRange)*rand(1);
                else
                    error('un-recognized type for next_side');
                end
                next_pole_XXXX = next_pole_angle;
                previous_pole_XXXX = previous_pole_angles;
            else
                error('trial type not ''r'' or ''a''')
            end


            if next_trial_type == 'a'
                previous_pole_radii(n_started_trials) = decisionBoundaryRadialDistance;
                previous_pole_angles(n_started_trials) = next_pole_angle;
                next_pole_position_main = round(center_main(:) - radius * sind(next_pole_angle));
                next_pole_position_lat = round(center_lat(:) - radius * cosd(next_pole_angle));
            elseif next_trial_type == 'r'
                previous_pole_radii(n_started_trials) = next_pole_radius;
                previous_pole_angles(n_started_trials) = decisionBoundaryAngle;
                next_pole_position_main = round(center_main(:) - next_pole_radius * sind(decisionBoundaryAngle));
                next_pole_position_lat = round(center_lat(:) - next_pole_radius * cosd(decisionBoundaryAngle));
            else
                error('trial type not ''r'' or ''a''')
            end

            %%%resample if uneven sample is triggered start%%%
            if numel(previous_sides)>=3
                if strcmp(value(sample_mode), 'adaptive') || strcmp(value(sample_mode), 'threshold')
                    posPerAxis = posSampPerAxis;
                    increments = range/posPerAxis;
                    samplingArray = (startRange):increments:(endRange);
                    samplingArray(end) = ceil(samplingArray(end));%###cant use ceil anf floor here casue very small increments makes these values rise up alot
                    samplingArray(1) = floor(samplingArray(1));
                    %              previous_pole_angles
                    inRangeXXXX = previous_pole_XXXX(logical((previous_trial_types==next_trial_type).*(previous_sides==next_side)));
                    inRangeIndex  = (inRangeXXXX>=startRange).*(inRangeXXXX<=endRange);
                    inRangeXXXX = inRangeXXXX(inRangeIndex==1);
                    if isempty(inRangeXXXX)
                        inRangeXXXX = [0];
                    end
                    distributionOfSampledPositions = histc(inRangeXXXX, samplingArray);
                    distributionOfSampledPositions = (distributionOfSampledPositions/(posPerAxis))/(1/(posPerAxis));
                    distributionOfSampledPositions = distributionOfSampledPositions(1:end-1);

                    samplingCount = histc(next_pole_XXXX, samplingArray);
                    samplingCount = samplingCount(1:end-1);
                    index = find(samplingCount);
                    meanPositionsPerBin = numel(inRangeXXXX)/posPerAxis;
                    percentSampPositions = (distributionOfSampledPositions/meanPositionsPerBin)/sampBiasThreshold;

                    if strcmp(value(sample_mode), 'adaptive')
                        equalizerModeSet = percentSampPositions(index);
                    else strcmp(value(sample_mode), 'threshold')
                        equalizerModeSet = - 1;%set to never trigger the 2nd part of the below OR statement
                    end
                    count = 1;
                    if ~isempty(percentSampPositions)
                        while percentSampPositions(index)> 1 || equalizerModeSet > rand()
                            count = count +1;
                            if next_trial_type == 'a'
                                next_pole_angle = endRange + (startRange-endRange)*rand(1);
                                next_pole_XXXX = next_pole_angle;
                            elseif next_trial_type == 'r'
                                next_pole_radius = round(startRange + rand*(range));
                                next_pole_XXXX = next_pole_radius;
                            else
                                error('trial type not ''r'' or ''a''')
                            end

                            samplingCount = histc(next_pole_XXXX, samplingArray);
                            samplingCount = samplingCount(1:end-1);
                            index = find(samplingCount);
                            percentSampPositions = (distributionOfSampledPositions/meanPositionsPerBin)/sampBiasThreshold;
                            equalizerModeSet = percentSampPositions(index);
                            if (count/50) == round(count/50)
                                'hey'
                                count
                            end
                        end
                    end
                elseif strcmp(value(sample_mode), 'off')
                    %dont do anything -- let randomness take over
                end
            end
            %%%resample if uneven sample is triggered END%%%%%



            antGOposition_main = center_main(:) - radius * sind(angle_startTOantGO);
            antGOposition_lat = center_lat(:) - radius * cosd(angle_startTOantGO);
            antGAPposition_main = center_main(:) - radius * sind(angle_startTOantGAP);%same as post nogo position
            antGAPposition_lat = center_lat(:) - radius * cosd(angle_startTOantGAP);
            half_point_main = round(value(antGAPposition_main + antGOposition_main)/2);
            half_point_lat  = round(value(antGAPposition_lat + antGOposition_lat)/2);


            next_pole_position_main
            next_pole_position_lat
            tic

            move_absolute_sequence3(motors,{half_point_main,next_pole_position_main},value(motor_num),...
                {half_point_lat,next_pole_position_lat},value(lateral_motor_num));
            movetime = toc
            if movetime<value(motor_move_time) % Should make this min-ITI a SoloParamHandle
                pause( value(motor_move_time)-movetime);
            end

            MotorsSection(obj,'read_positions');
            trial_ready_times.value = clock;

            previous_pole_positions(n_started_trials) = next_pole_position_main;
            previous_pole_positions_lat(n_started_trials) = next_pole_position_lat;









            %----------GRID OFF AXES------------

        elseif strcmp(value(position_mode), 'grid_off_axes')
            %to be implemented for testing positions outside of the axes
            %(different arc lengths probing same theta at differnet radial
            %distances.

            %-----------ARC ------------------
        elseif strcmp(value(position_mode), 'arc')%%%ARC pole mode
            % y =  post_go_lat(:)
            % x =  post_go(:)
            % h = center_main(:)
            % k = center_lat(:)
            AddedAngle = 0; %in case you need to rotate the start angle. set to 0 otherwise.


            radius = sqrt((post_go(:)-center_main(:))^2 + (post_go_lat(:)-center_lat(:))^2);
            ArcLengthGO =   2*pi*radius*go_degrees/360;
            ArcLengthGAP =  2*pi*radius*gap_degrees/360;
            ArcLengthNOGO = 2*pi*radius*nogo_degrees/360;

            % y1 =  post_go_lat(:)
            % x1 =  post_go(:)

            %x2 = center_main(:)
            %y2 = center_lat(:)

            length = center_main(:) - post_go(:);
            height = center_lat(:) - post_go_lat(:);

            lengthBase = sqrt(length^2 + height^2);
            halfBase = lengthBase/2;

            %in degrees
            angleToPostGoPosition = asind(length/radius);


            if next_side == 'r'
                next_pole_angle = go_degrees(:)*((round(rand*(ArcLengthGO)))/ArcLengthGO)+angleToPostGoPosition+AddedAngle;

                next_pole_position_main = round(center_main(:) - radius * sind(next_pole_angle));
                next_pole_position_lat = round(center_lat(:) - radius * cosd(next_pole_angle));


            elseif next_side == 'l'
                next_pole_angle = nogo_degrees(:)*((round(rand*(ArcLengthNOGO)))/ArcLengthNOGO)...
                    +angleToPostGoPosition+go_degrees+gap_degrees+AddedAngle;

                next_pole_position_main = round(center_main(:) - radius * sind(next_pole_angle));
                next_pole_position_lat = round(center_lat(:) - radius * cosd(next_pole_angle));


            else
                error('un-recognized type for next_side');
            end
            %angleToPostGoPosition
            angle_startTOantGO = angleToPostGoPosition + go_degrees(:)+AddedAngle;
            angle_startTOantGAP = angle_startTOantGO + gap_degrees(:);
            angle_startTOantNOGO = angle_startTOantGAP + nogo_degrees(:);



            if angle_startTOantNOGO > 95 || angle_startTOantNOGO < -50
                while angle_startTOantNOGO > 95 || angle_startTOantNOGO < -50
                    warning('your angle from vertical downward axis (negative y axis from center)is more than 95 or less than -50. this might harm the mouse please change or override')
                    angle_startTOantGO = angleToPostGoPosition + go_degrees(:)+AddedAngle;
                    angle_startTOantGAP = angle_startTOantGO + gap_degrees(:);
                    angle_startTOantNOGO = angle_startTOantGAP + nogo_degrees(:);
                    '10 sec pause while greater than 95 or less than -50'
                    pause(10);
                end
            end
            antGOposition_main = center_main(:) - radius * sind(angle_startTOantGO);
            antGOposition_lat = center_lat(:) - radius * cosd(angle_startTOantGO);

            antGAPposition_main = center_main(:) - radius * sind(angle_startTOantGAP);%same as post nogo position
            antGAPposition_lat = center_lat(:) - radius * cosd(angle_startTOantGAP);


            half_point_main = round(value(antGAPposition_main + antGOposition_main)/2);
            half_point_lat  = round(value(antGAPposition_lat + antGOposition_lat)/2);


            %check all this I am just extrapolating from what I see here make sure it
            %translates
            next_pole_position_main
            next_pole_position_lat
            tic
            %             move_absolute_sequence(motors,{half_point_main,next_pole_position_main},value(motor_num));
            %             move_absolute_sequence(motors,{half_point_lat,next_pole_position_lat},value(lateral_motor_num));
            move_absolute_sequence3(motors,{half_point_main,next_pole_position_main},value(motor_num),...
                {half_point_lat,next_pole_position_lat},value(lateral_motor_num));
            movetime = toc
            if movetime<value(motor_move_time) % Should make this min-ITI a SoloParamHandle
                pause( value(motor_move_time)-movetime);
            end

            MotorsSection(obj,'read_positions');
            trial_ready_times.value = clock;

            previous_pole_positions(n_started_trials) = next_pole_position_main;
            previous_pole_positions_lat(n_started_trials) = next_pole_position_lat;

            %---------NORMAL MODE----------
        elseif strcmp(value(position_mode), 'normal')%%%normal pole mode
            if next_side == 'r'
                next_pole_position = value(round(rand*(yes_pole_position_ant - yes_pole_position_pos)+yes_pole_position_pos));
            elseif next_side == 'l'
                next_pole_position = value(round(rand*(no_pole_position_ant - no_pole_position_pos)+no_pole_position_pos));
            else
                error('un-recognized type for next_side');
            end

            half_point = round(value(no_pole_position_pos+yes_pole_position_ant)/2);

            tic
            move_absolute_sequence(motors,{half_point,next_pole_position},value(motor_num));
            movetime = toc
            if movetime<value(motor_move_time) % Should make this min-ITI a SoloParamHandle
                pause( value(motor_move_time)-movetime);
            end


            previous_pole_radii(n_started_trials+1)  = sqrt(abs((next_pole_position-center_main(:))^2) + abs((normLatMotPositionSET(:)-center_lat(:))^2));
            previous_pole_angles(n_started_trials+1) = asind((center_main(:)-next_pole_position(:)) /  previous_pole_radii(n_started_trials+1));


            MotorsSection(obj,'read_positions');
            trial_ready_times.value = clock;

            previous_pole_positions(n_started_trials) = next_pole_position;
            previous_pole_positions_lat(n_started_trials) = value(lateral_motor_position);
        end
        return;



    case 'get_previous_pole_position',   % --------- CASE get_next_pole_position ------
        if isempty(value(previous_pole_positions))
            x = nan;
        else
            x = previous_pole_positions(length(previous_pole_positions));
        end
        return;

    case 'get_all_previous_pole_positions',   % --------- CASE get_next_pole_position ------
        x = value(previous_pole_positions);
        return;

    case 'get_yes_pole_position_easy'
        x = value(yes_pole_position_ant);
        return

    case 'get_no_pole_position_easy'
        x = value(no_pole_position_pos);
        return

    case 'get_num_of_pole_position'

        x = 1;

        return




    case 'motors_home',     %modified by ZG 10/1/11
        %         disp(motors);
        %         disp(value(motor_num));
        move_home(motors, value(motor_num));
        return;

    case 'serial_open',
        serial_open(motors);
        return;

    case 'serial_reset',
        close_and_cleanup(motors);

        global motors_properties;
        global motors;

        if strcmp(motors_properties.type,'@FakeZaberAMCB2')
            motors = FakeZaberAMCB2;
        else
            motors = ZaberAMCB2;
        end

        serial_open(motors);
        return;

    case 'motors_stop',
        stop(motors);
        return;

    case 'motors_reset',
        reset(motors);
        return;

    case 'reset_motors_firmware',
        set_initial_parameters(motors)
        display('Reset speed, acceleration, and motor bus ID numbers.')
        return;

    case 'motor_position',
        position = value(motor_position);
        if position > value(motor_max_position) | position < 0
            p = get_position(motors,value(motor_num));
            motor_position.value = p;
        else
            move_absolute(motors,position,value(motor_num));
        end
        return;

    case 'lateral_motor_position',
        position = value(lateral_motor_position);
        if position > value(motor_max_position) | position < 0
            p = get_position(motors,value(lateral_motor_num));
            lateral_motor_position.value = p;
        else
            move_absolute(motors,position,value(lateral_motor_num));
        end
        return;

    case 'normLatMotPositionSET',
        %this is a user set value. set to lateral position for linear
        %'normal' task to be used for calculating range of ragial
        %distancein the radial distance task(s)-PSM
        return;


    case 'read_positions'
        p = get_position(motors,value(motor_num));
        motor_position.value = p;
        return;

    case 'read_lateral_positions'
        p = get_position(motors,value(lateral_motor_num));
        lateral_motor_position.value = p;
        return;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%-PSM joystick
    case 'move_right'
        position = value(motor_position)+joystick_increment;
        if position > value(motor_max_position)
            position = value(motor_max_position);
        elseif position < 0
            position = 0;
        end
        move_absolute(motors,position,value(motor_num));
        motor_position.value = position;
        return;
    case 'move_left'
        position = value(motor_position)-joystick_increment;
        if position > value(motor_max_position)
            position = value(motor_max_position);
        elseif position < 0
            position = 0;
        end
        move_absolute(motors,position,value(motor_num));
        motor_position.value = position;
        return;
    case 'move_up',
        position = value(lateral_motor_position)+joystick_increment;
        if position > value(motor_max_position)
            position = value(motor_max_position);
        elseif position < 0
            position = 0;
        end
        move_absolute(motors,position,value(lateral_motor_num));
        lateral_motor_position.value = position;
        return;

    case 'move_down',
        position = value(lateral_motor_position)-joystick_increment;
        if position > value(motor_max_position)
            position = value(motor_max_position);
        elseif position < 0
            position = 0;
        end
        move_absolute(motors,position,value(lateral_motor_num));
        lateral_motor_position.value = position;
        return;

    case 'set_both_post_go'
        post_go_lat.value = round(lateral_motor_position(:));
        post_go.value = round(motor_position(:));
        return;

    case 'set_center'
        center_lat.value = round(lateral_motor_position(:));
        center_main.value = round(motor_position(:));
        return;
        %%%%%%%%%%%%%%%%%%%%%%%%%%-PSM joystick

    case 'show_nogo_arc'
        deg_steps = round(nogo_degrees(:)/3);
        arc_array = (angle_startTOantGAP:(nogo_degrees(:)/deg_steps):angle_startTOantNOGO);
        arc_array = [arc_array,fliplr(arc_array)];
        show_arc_main = round(center_main(:) - radius * sind(arc_array));
        show_arc_lat = round(center_lat(:) - radius * cosd(arc_array));

        move_absolute_sequence3(motors,{show_arc_main(1)},value(motor_num),...
            {show_arc_lat(1)},value(lateral_motor_num));
        pause(1.5);
        NumArcArray = numel(arc_array);
        for D = 1:NumArcArray
            move_absolute_sequence3(motors,{show_arc_main(D)},value(motor_num),...
                {show_arc_lat(D)},value(lateral_motor_num));
            pause(.01)
        end
        for D =1:2
            move_absolute_sequence3(motors,{show_arc_main(NumArcArray/2)},value(motor_num),...
                {show_arc_lat(NumArcArray/2)},value(lateral_motor_num));
            pause(.5)
            move_absolute_sequence3(motors,{show_arc_main(1)},value(motor_num),...
                {show_arc_lat(1)},value(lateral_motor_num));
            pause(.5)

        end
        return;
    case 'show_gap_arc'
        deg_steps = round(gap_degrees(:)/3);
        arc_array = (angle_startTOantGO:(gap_degrees(:)/deg_steps):angle_startTOantGAP);
        arc_array = [arc_array,fliplr(arc_array)];
        show_arc_main = round(center_main(:) - radius * sind(arc_array));
        show_arc_lat = round(center_lat(:) - radius * cosd(arc_array));

        move_absolute_sequence3(motors,{show_arc_main(1)},value(motor_num),...
            {show_arc_lat(1)},value(lateral_motor_num));
        pause(1.5);
        NumArcArray = numel(arc_array);
        for D = 1:NumArcArray
            move_absolute_sequence3(motors,{show_arc_main(D)},value(motor_num),...
                {show_arc_lat(D)},value(lateral_motor_num));
            pause(.01)
        end
        for D =1:2
            move_absolute_sequence3(motors,{show_arc_main(NumArcArray/2)},value(motor_num),...
                {show_arc_lat(NumArcArray/2)},value(lateral_motor_num));
            pause(.5)
            move_absolute_sequence3(motors,{show_arc_main(1)},value(motor_num),...
                {show_arc_lat(1)},value(lateral_motor_num));
            pause(.5)

        end
        return;
    case 'show_go_arc'
        deg_steps = round(go_degrees(:)/3);
        arc_array = (angleToPostGoPosition:(go_degrees(:)/deg_steps):angle_startTOantGO);
        arc_array = [arc_array,fliplr(arc_array)];
        show_arc_main = round(center_main(:) - radius * sind(arc_array));
        show_arc_lat = round(center_lat(:) - radius * cosd(arc_array));

        move_absolute_sequence3(motors,{show_arc_main(1)},value(motor_num),...
            {show_arc_lat(1)},value(lateral_motor_num));
        pause(1.5);
        NumArcArray = numel(arc_array);
        for D = 1:NumArcArray
            move_absolute_sequence3(motors,{show_arc_main(D)},value(motor_num),...
                {show_arc_lat(D)},value(lateral_motor_num));
            pause(.01)
        end
        for D =1:2
            move_absolute_sequence3(motors,{show_arc_main(NumArcArray/2)},value(motor_num),...
                {show_arc_lat(NumArcArray/2)},value(lateral_motor_num));
            pause(.5)
            move_absolute_sequence3(motors,{show_arc_main(1)},value(motor_num),...
                {show_arc_lat(1)},value(lateral_motor_num));
            pause(.5)

        end

        return;
        % --------- CASE HIDE_SHOW ---------------------------------

    case 'hide_show'
        if strcmpi(value(motor_show), 'hide')
            set(value(motorfig), 'Visible', 'off');
        elseif strcmpi(value(motor_show),'view')
            set(value(motorfig),'Visible','on');
        end;
        return;


    case 'reinit',   % ------- CASE REINIT -------------
        currfig = gcf;

        % Get the original GUI position and figure:
        x = my_gui_info(1); y = my_gui_info(2); figure(my_gui_info(3));

        delete(value(myaxes));

        % Delete all SoloParamHandles who belong to this object and whose
        % fullname starts with the name of this mfile:
        delete_sphandle('owner', ['^@' class(obj) '$'], ...
            'fullname', ['^' mfilename]);

        % Reinitialise at the original GUI position and figure:
        [x, y] = feval(mfilename, obj, 'init', x, y);

        % Restore the current figure:
        figure(currfig);
        return;
end



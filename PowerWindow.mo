package PowerWindow
  import Modelica.Blocks.Interfaces;
  type TriState = Integer(min = -1, max = 1);
  type ControllerStates = enumeration(Stop, MoveUp, ManualMoveUp, AutoMoveUp, EmergencyDown, MoveDown, ManualMoveDown, AutoMoveDown);
  connector TriStateInput = input TriState "'input TriState data' as connector" annotation(defaultComponentName = "u", Icon(graphics = {Polygon(points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}, lineColor = {255, 127, 0}, fillColor = {255, 127, 0}, fillPattern = FillPattern.Solid)}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.2)), Diagram(coordinateSystem(preserveAspectRatio = true, initialScale = 0.2, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{0, 50}, {100, 0}, {0, -50}, {0, 50}}, lineColor = {255, 127, 0}, fillColor = {255, 127, 0}, fillPattern = FillPattern.Solid), Text(extent = {{-10, 85}, {-10, 60}}, lineColor = {255, 127, 0}, textString = "%name")}), Documentation(info = "<html>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        <p>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        Connector with one input signal of Tri-State type.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        </p>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        </html>"));
  connector TriStateOutput = output TriState "'output TriState data' as connector" annotation(defaultComponentName = "y", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-100, 100}, {100, 0}, {-100, -100}, {-100, 100}}, lineColor = {255, 127, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-100, 50}, {0, 0}, {-100, -50}, {-100, 50}}, lineColor = {255, 127, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{30, 110}, {30, 60}}, lineColor = {255, 127, 0}, textString = "%name")}), Documentation(info = "<html>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        <p>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        Connector with one output signal of Tri-state type.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        </p>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        </html>"));

  model PWControl
    //Controller state enumeration variable
    ControllerStates state(start = ControllerStates.Stop);
    //The time interval after which a time-event is being scheduled
    Real eventInterval(start = 0);
    //The time instant at which next time-event is scheduled
    Real eventScheduleInstant(start = 0);
    output PowerWindow.TriStateOutput motor(start = 0) annotation(Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    input PowerWindow.TriStateInput windowState(start = -1) annotation(Placement(visible = true, transformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    input Modelica.Blocks.Interfaces.BooleanInput obstacle annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    input PowerWindow.TriStateInput cmd(start = 0) annotation(Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  initial equation
    pre(cmd) = 0;
  algorithm
    //if scheduled Event then
    when time >= eventScheduleInstant + eventInterval then
      if state == ControllerStates.MoveUp then
        state := ControllerStates.ManualMoveUp;
      elseif state == ControllerStates.MoveDown then
        state := ControllerStates.ManualMoveDown;
      elseif state == ControllerStates.EmergencyDown then
        state := ControllerStates.Stop;
        motor := 0;
      end if;
    end when;
    //When an obstacle is detected
    when obstacle then
      if state == ControllerStates.ManualMoveUp or state == ControllerStates.AutoMoveUp then
        state := ControllerStates.EmergencyDown;
        motor := -1;
        eventScheduleInstant := time;
        eventInterval := 2;
      end if;
    elsewhen change(cmd) then
      if state == ControllerStates.Stop then
        if cmd == 1 then
          motor := 1;
          eventScheduleInstant := time;
          state := ControllerStates.MoveUp;
          eventInterval := 0.02;
        elseif cmd == (-1) then
          motor := -1;
          eventScheduleInstant := time;
          state := ControllerStates.MoveDown;
          eventInterval := 0.02;
        end if;
      elseif state == ControllerStates.MoveUp then
        if cmd == 0 then
          state := ControllerStates.AutoMoveUp;
          eventInterval := 0;
        elseif cmd == (-1) then
          state := ControllerStates.MoveDown;
          motor := -1;
        end if;
      elseif state == ControllerStates.ManualMoveUp then
        if cmd == 0 then
          state := ControllerStates.Stop;
          motor := 0;
        elseif cmd == (-1) then
          state := ControllerStates.MoveDown;
          motor := -1;
        end if;
      elseif state == ControllerStates.AutoMoveUp then
        state := ControllerStates.Stop;
        motor := 0;
      elseif state == ControllerStates.MoveDown then
        if cmd == 0 then
          state := ControllerStates.AutoMoveDown;
          eventInterval := 0;
        elseif cmd == 1 then
          state := ControllerStates.MoveUp;
        end if;
      elseif state == ControllerStates.ManualMoveDown then
        if cmd == 0 then
          state := ControllerStates.Stop;
          motor := 0;
        elseif cmd == 1 then
          state := ControllerStates.MoveUp;
          motor := 1;
        end if;
      elseif state == ControllerStates.AutoMoveDown then
        state := ControllerStates.Stop;
        motor := 0;
      end if;
    end when;
    //when Window State is "Completely Up"
    when windowState == 1 then
      if state == ControllerStates.AutoMoveUp then
        state := ControllerStates.Stop;
        motor := 0;
      end if;
    end when;
    //When Window State is "Completely Down"
    when windowState == (-1) then
      if state == ControllerStates.AutoMoveDown then
        state := ControllerStates.Stop;
        motor := 0;
      end if;
    end when;
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Text(origin = {8.932829999999999, -4.25148}, lineColor = {170, 0, 0}, extent = {{-68.38, 24.75}, {54.03, -12.74}}, textString = "Power Window Controller"), Rectangle(origin = {-0.293042, 2.1987}, fillColor = {85, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-74.38, 78.92}, {74.38, -78.92}})}), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Rectangle(origin = {1.60565, -6.88521}, fillColor = {85, 170, 255}, fillPattern = FillPattern.Solid, extent = {{-83.0187, 84.1859}, {82.14019999999999, -78.91500000000001}}), Text(origin = {-4.23805, 27.3769}, lineColor = {170, 0, 0}, extent = {{-43.78, 15.67}, {52.8613, -8.051769999999999}}, textString = "Power", textStyle = {TextStyle.Bold}), Text(origin = {-3.27413, -3.57716}, lineColor = {170, 0, 0}, extent = {{-43.78, 15.67}, {52.86, -8.050000000000001}}, textString = "Window", textStyle = {TextStyle.Bold}), Text(origin = {-1.72413, -33.6533}, lineColor = {170, 0, 0}, extent = {{-43.78, 15.67}, {52.86, -8.050000000000001}}, textString = "Controller", textStyle = {TextStyle.Bold})}));
  end PWControl;

  model PW_ElectroMechSystem
    parameter Real highEndStop(start = 5.0);
    parameter Real lowEndStop(start = 0.0);
    PowerWindow.TriStateOutput endStopOut annotation(Placement(visible = true, transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Blocks.Nonlinear.Limiter limiter1(uMax = highEndStop, uMin = lowEndStop) annotation(Placement(visible = true, transformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Blocks.Logical.LessEqual lessequal1 annotation(Placement(visible = true, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Blocks.Logical.GreaterEqual greaterequal1 annotation(Placement(visible = true, transformation(origin = {0, 3}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Blocks.Sources.Constant highEndStopConst(k = highEndStop) annotation(Placement(visible = true, transformation(origin = {-60, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Blocks.Sources.Constant lowEndStopConst(k = lowEndStop) annotation(Placement(visible = true, transformation(origin = {-60, -49}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    input PowerWindow.TriStateInput motorIn annotation(Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain1(k = 2) annotation(Placement(visible = true, transformation(origin = {-55, 45}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.IntegerToReal integertoreal1 annotation(Placement(visible = true, transformation(origin = {-74, 45}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    inner Modelica.Blocks.Math.Add add1(k2 = -1) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToInteger booleantointeger2 annotation(Placement(visible = true, transformation(origin = {25, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.BooleanToInteger booleantointeger1 annotation(Placement(visible = true, transformation(origin = {26, 3}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.IntegerToReal integertoreal2 annotation(Placement(visible = true, transformation(origin = {40, 3}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.IntegerToReal integertoreal3 annotation(Placement(visible = true, transformation(origin = {40, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    Modelica.Blocks.Math.RealToInteger realtointeger2 annotation(Placement(visible = true, transformation(origin = {80, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    inner Modelica.Blocks.Continuous.LimIntegrator limintegrator1(outMax = 10, outMin = 0, limitsAtInit = true) annotation(Placement(visible = true, transformation(origin = {-29, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput wndPositionOut annotation(Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(limiter1.y, wndPositionOut) annotation(Line(points = {{31, 40}, {69.0265, 40}, {69.0265, 18.7611}, {90.6195, 18.7611}, {90.6195, 18.7611}}));
    connect(limintegrator1.y, lessequal1.u1) annotation(Line(points = {{-18, 40}, {-11.8394, 40}, {-11.8394, 16.4597}, {-26.2778, 16.4597}, {-26.2778, -40.4274}, {-11.5507, -40.4274}, {-11.5507, -40.4274}}));
    connect(limintegrator1.y, greaterequal1.u1) annotation(Line(points = {{-18, 40}, {-11.8394, 40}, {-11.8394, 16.1709}, {-26.2778, 16.1709}, {-26.2778, 3.4652}, {-12.7057, 3.4652}, {-12.7057, 3.4652}}));
    connect(gain1.y, limintegrator1.u) annotation(Line(points = {{-49.5, 45}, {-45.9139, 45}, {-45.9139, 40.1386}, {-42.4487, 40.1386}, {-42.4487, 40.1386}}));
    connect(limintegrator1.y, limiter1.u) annotation(Line(points = {{-18, 40}, {7.90139, 40}, {7.90139, 40.0713}, {7.90139, 40.0713}}));
    connect(realtointeger2.y, endStopOut) annotation(Line(points = {{85.5, -40}, {89.8203, -40}, {89.8203, -40.08}, {89.8203, -40.08}}));
    connect(add1.y, realtointeger2.u) annotation(Line(points = {{71, -20}, {73.37730000000001, -20}, {73.37730000000001, -33.5028}, {63.5114, -33.5028}, {63.5114, -40.2856}, {73.37730000000001, -40.2856}, {73.37730000000001, -40.2856}}));
    connect(integertoreal3.y, add1.u2) annotation(Line(points = {{45.5, -40}, {48.7126, -40}, {48.7126, -30.8308}, {35.9692, -30.8308}, {35.9692, -26.1034}, {46.2462, -26.1034}, {46.2462, -26.5145}, {46.2462, -26.5145}}));
    connect(booleantointeger2.y, integertoreal3.u) annotation(Line(points = {{30.5, -40}, {34.1194, -40}, {34.1194, -40.08}, {34.1194, -40.08}}));
    connect(integertoreal2.y, add1.u1) annotation(Line(points = {{45.5, 3}, {47.6849, 3}, {47.6849, -6.78277}, {37.408, -6.78277}, {37.408, -13.9766}, {46.0406, -13.9766}, {46.0406, -13.9766}}));
    connect(booleantointeger1.y, integertoreal2.u) annotation(Line(points = {{31.5, 3}, {33.7083, 3}, {33.7083, 2.87754}, {33.7083, 2.87754}}));
    connect(greaterequal1.y, booleantointeger1.u) annotation(Line(points = {{11, 3}, {19.1151, 3}, {19.1151, 2.87754}, {19.1151, 2.87754}}));
    connect(lessequal1.y, booleantointeger2.u) annotation(Line(points = {{11, -40}, {24.1956, -40}, {19, -40.1219}, {19, -40}}));
    connect(integertoreal1.y, gain1.u) annotation(Line(points = {{-68.5, 45}, {-61.0687, 45}, {-61.0687, 44.7837}, {-61.0687, 44.7837}}));
    connect(motorIn, integertoreal1.u) annotation(Line(points = {{-100, 40}, {-87.5318, 40}, {-87.5318, 44.7837}, {-79.38930000000001, 44.7837}, {-79.38930000000001, 44.7837}}));
    connect(lowEndStopConst.y, lessequal1.u2) annotation(Line(points = {{-49, -49}, {-32.7713, -49}, {-32.7713, -47.4724}, {-12.8635, -47.4724}, {-12.8635, -47.4724}, {-12.8635, -47.4724}}));
    connect(highEndStopConst.y, greaterequal1.u2) annotation(Line(points = {{-49, -10}, {-29.096, -10}, {-29.096, -4.90038}, {-12.8635, -4.90038}, {-12.8635, -4.90038}}));
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Rectangle(origin = {-0.234742, -3.28638}, extent = {{-84.2723, 81.6901}, {83.33329999999999, -52.5822}}), Rectangle(origin = {-57.0405, 44.1295}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {-58.1673, -38.2179}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {4.55573, 3.84779}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {-25.679, -21.6921}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {33.4759, 29.4816}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {35.1661, -23.7578}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {63.6168, -40.847}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {63.3351, 42.4393}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Rectangle(origin = {63.1473, -0.940939}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Line(origin = {-78.169, 42.723}, points = {{-12.6798, -42.7014}, {1.8742, -42.7014}, {2.58216, 2.34742}, {11.9718, 2.34742}}), Line(origin = {-41.0798, 37.3239}, points = {{-7.277, 7.277}, {2.11268, 7.277}, {2.11268, -31.2207}, {35.9155, -31.2207}, {36.385, -31.2207}}), Line(origin = {-41.7442, -30.5182}, points = {{-7.08205, -7.51001}, {1.36866, -7.51001}, {1.36866, 7.51346}, {7.00246, 7.51346}}), Rectangle(origin = {-14.0358, 43.0027}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-9.15, 5.63}, {9.15, -5.63}}), Line(origin = {-31.2207, 44.1315}, points = {{-7.74648, 0}, {7.74648, 0}}), Line(origin = {25.1174, 43.662}, points = {{-29.8122, 0}, {29.8122, 0}}), Line(origin = {-10.5634, -10.7981}, points = {{-5.39906, -11.2676}, {1.64319, -11.2676}, {1.64319, 11.2676}, {5.39906, 11.2676}}), Line(origin = {19.0141, 17.1362}, points = {{-5.39906, -13.3803}, {1.64319, -13.3803}, {1.64319, 13.3803}, {5.39906, 13.3803}}), Line(origin = {23.4742, -10.0939}, points = {{-2.8169, 13.3803}, {-2.8169, -13.3803}, {2.8169, -13.3803}}), Line(origin = {47.9455, 34.5032}, points = {{-5.22248, -5.3952}, {1.35029, -5.3952}, {1.35029, 5.40292}, {6.51461, 5.40292}}), Line(origin = {49.2958, -11.7371}, points = {{-5.16432, -11.7371}, {0.938967, -11.7371}, {0.938967, 11.7371}, {5.16432, 11.7371}}), Line(origin = {52.3474, -32.1596}, points = {{-2.11268, 8.685449999999999}, {-2.11268, -8.685449999999999}, {2.11268, -8.685449999999999}}), Line(origin = {80.3935, 38.8367}, points = {{-8.685449999999999, 1.40845}, {-2.11268, 1.40845}, {-2.11268, -19.4615}, {9.98587, -19.4615}}), Line(origin = {82.1596, -40.3756}, points = {{-9.389670000000001, 0}, {9.389670000000001, 0}}), Text(origin = {-0.47, -74.88}, extent = {{-84.51000000000001, 13.85}, {84.51000000000001, -13.85}}, textString = "PW Elctro Mech Sys")}));
  end PW_ElectroMechSystem;

  model PowerWindowModel
    input PowerWindow.TriStateInput cmdInput annotation(Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerWindow.PW_ElectroMechSystem pw_electromechsystem(highEndStop = 5, lowEndStop = 0) annotation(Placement(visible = true, transformation(origin = {45, 15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput wndPositionOut annotation(Placement(visible = true, transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerWindow.TriStateOutput wndEndStopOut annotation(Placement(visible = true, transformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    input Modelica.Blocks.Interfaces.BooleanInput obstacleInput annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerWindow.TriStateOutput cntrMotorOut annotation(Placement(visible = true, transformation(origin = {100, 51}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    PowerWindow.PWControl pwcontrol1 annotation(Placement(visible = true, transformation(origin = {-20, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(pwcontrol1.motor, cntrMotorOut) annotation(Line(points = {{-10, 20}, {11.1922, 20}, {11.1922, 50.6083}, {90.2676, 50.6083}, {90.2676, 50.6083}}));
    connect(pw_electromechsystem.endStopOut, pwcontrol1.windowState) annotation(Line(points = {{60, 9}, {66.4234, 9}, {66.4234, -7.54258}, {-40.8759, -7.54258}, {-40.8759, 16.0584}, {-30.6569, 16.0584}, {-30.6569, 16.0584}}));
    connect(pwcontrol1.motor, pw_electromechsystem.motorIn) annotation(Line(points = {{-10, 20}, {12.8954, 20}, {12.8954, 14.8418}, {29.4404, 14.8418}, {29.4404, 14.8418}}));
    connect(obstacleInput, pwcontrol1.obstacle) annotation(Line(points = {{-100, 0}, {-54.9878, 0}, {-54.9878, 19.708}, {-30.4136, 19.708}, {-30.4136, 19.708}}));
    connect(cmdInput, pwcontrol1.cmd) annotation(Line(points = {{-100, 40}, {-62.2871, 40}, {-62.2871, 23.8443}, {-29.6837, 23.8443}, {-29.6837, 23.8443}}));
    connect(pw_electromechsystem.endStopOut, wndEndStopOut) annotation(Line(points = {{60, 9}, {80, 9}, {80, -20.177}, {89.5575, -20.177}, {89.5575, -20.177}}));
    connect(pw_electromechsystem.wndPositionOut, wndPositionOut) annotation(Line(points = {{60, 18}, {81.41589999999999, 18}, {81.41589999999999, 19.469}, {90.6195, 19.469}, {90.6195, 19.469}}));
    annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Rectangle(origin = {0.35, -19.47}, lineColor = {85, 170, 255}, fillColor = {0, 110, 165}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-61.59, 31.5}, {61.59, -31.5}}), Polygon(origin = {1.32, 38.8199}, fillPattern = FillPattern.Sphere, points = {{-62.5544, -26.3083}, {-29.988, 26.789}, {62.4013, 26.789}, {58.8615, -27.7242}, {58.8615, -25.6003}, {58.5075, -26.3083}, {-62.5544, -26.3083}}), Polygon(origin = {-0.0902655, 38.465}, fillColor = {189, 189, 189}, fillPattern = FillPattern.Solid, points = {{-44.8553, -22.0605}, {-25.7403, 23.6032}, {58.8615, 23.6032}, {51.4279, -22.4145}, {51.0739, -22.4145}, {50.7199, -22.4145}, {-44.8553, -22.0605}}), Rectangle(origin = {45.13, 3.19}, fillColor = {85, 170, 127}, fillPattern = FillPattern.Solid, extent = {{-9.380000000000001, 2.12}, {9.380000000000001, -2.12}})}));
  end PowerWindowModel;
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), uses(Modelica(version = "2.2.2")));
end PowerWindow;
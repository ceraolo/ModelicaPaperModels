package VSC_GFL_GFM "Models of different VSCs, grid forming and grid following. Test cases in MV-LV grid."
  package Blocks
    model Q_f
      Modelica.Blocks.Interfaces.RealInput u[6] annotation(
        Placement(transformation(origin = {-106, 2}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-104, 14}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
    equation
      y = (-u[1]*u[5] + u[2]*u[4]);
      annotation(
        Diagram(graphics = {Text(origin = {0, 41}, extent = {{-52, 19}, {52, -19}}, textString = "y[1]=(-u[1]*u[5]+u[2]*u[4]);

 Q = - Vd Iq + Vq Id", textColor = {0, 0, 0})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end Q_f;

    model P_f
      Modelica.Blocks.Interfaces.RealInput u[6] annotation(
        Placement(transformation(origin = {-106, 2}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-104, 14}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(transformation(origin = {106, 2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {106, 2}, extent = {{-10, -10}, {10, 10}})));
    equation
      y = (u[1]*u[4] + u[2]*u[5]);
      annotation(
        Diagram(graphics = {Text(origin = {12, 43}, extent = {{-52, 19}, {52, -19}}, textColor = {0, 0, 0}, textString = "y[1]=(u[1]*u[4]+u[2]*u[5]);

P = Vd Id + Vq Iq")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end P_f;

    model AwuFirstOrder
      import Modelica.Constants.pi;
      import Modelica.Constants.inf;
      parameter Real tau_v = 1/(4*2*pi*50) "time constant of feedforward vd e vq (s)";
      Modelica.Blocks.Interfaces.RealInput v_in annotation(
        Placement(transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-102, 0}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(transformation(extent = {{-50, -16}, {-30, 4}})));
      Modelica.Blocks.Math.Gain gain(k = 1/tau_v) annotation(
        Placement(transformation(extent = {{-2, -18}, {18, 2}})));
      Modelica.Blocks.Interfaces.RealOutput v_out annotation(
        Placement(transformation(origin = {108, -8}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {96, 0}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Continuous.LimIntegrator limIntegrator(outMax = inf, outMin = 0.01, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 1) annotation(
        Placement(transformation(extent = {{44, -18}, {64, 2}})));
    equation
      connect(add.u1, v_in) annotation(
        Line(points = {{-52, 0}, {-108, 0}}, color = {0, 0, 127}));
      connect(add.y, gain.u) annotation(
        Line(points = {{-29, -6}, {-16, -6}, {-16, -8}, {-4, -8}}, color = {0, 0, 127}));
      connect(v_out, v_out) annotation(
        Line(points = {{108, -8}, {108, -8}}, color = {0, 0, 127}));
      connect(gain.y, limIntegrator.u) annotation(
        Line(points = {{19, -8}, {42, -8}}, color = {0, 0, 127}));
      connect(limIntegrator.y, v_out) annotation(
        Line(points = {{65, -8}, {108, -8}}, color = {0, 0, 127}));
      connect(add.u2, limIntegrator.y) annotation(
        Line(points = {{-52, -12}, {-60, -12}, {-60, -24}, {76, -24}, {76, -8}, {65, -8}}, color = {0, 0, 127}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-126, 146}, {124, 108}}, textColor = {0, 0, 255}, textString = "%name"), Line(origin = {-26.667, 10.667}, points = {{106.667, 43.333}, {-13.333, 29.333}, {-53.333, -86.667}}, color = {0, 0, 127}, smooth = Smooth.Bezier), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{-80, 94}, {-88, 72}, {-72, 72}, {-80, 94}}), Line(points = {{-80, 82}, {-80, -86}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{90, -76}, {68, -68}, {68, -84}, {90, -76}}), Line(points = {{-90, -76}, {82, -76}}, color = {192, 192, 192}), Line(points = {{-18, 36}, {74, 36}}, color = {256, 0, 0}, thickness = 0.5)}),
        Diagram(coordinateSystem(preserveAspectRatio = false)),
        Documentation(info = "<html>
<p><span style=\"font-size: 9pt;\">Antiwindup-limited first-order delay</span></p>
</html>"));
    end AwuFirstOrder;

    model GfmDroopControl
      import Modelica.Constants.pi;
      import Modelica.Constants.inf;
      parameter Modelica.Units.SI.Frequency fNom = 50 "Nominal frequency";
      parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Nominal apparent power";
      parameter Modelica.Units.SI.Voltage uDc = 750 "Nominal DC voltage (used for per-unit)";
      parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph voltage (used for per-unit)";
      parameter Real pGain = 0.05 "P-loop control gain (PU)";
      parameter Real qGain = 0.05 "Q-loop control gain (PU)";
      parameter Modelica.Units.SI.Time fo = 0.02 "First order time constant for P and Q calculation in VSC control";
      parameter Real xcc = 0.058 "Xcc transfomer (PU), used to compensate transoformer voltage drop";
      parameter Real secReg = 1 "Secondary regulation, 1=ON 0=OFF";
      parameter Real secRegBand = 0.0002 "Secondary regulation dead semiband. total band = +-secRegBand (PU)";
      parameter Modelica.Units.SI.Time secRegResetDelay = 1 "If the frequency re-enters and remains within the dead-band for the specified period, secondary regulation is reset.";
      Modelica.Blocks.Interfaces.RealInput vabc[3] "Tensioni di fase ai morsetti del Vsc (V)" annotation(
        Placement(transformation(origin = {-129, 73}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput iabc[3] "Correnti di fase ai morsetti del Vsc (V)" annotation(
        Placement(transformation(origin = {-129, 41}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 28}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput pRef "Richiesta di potenza attiva (PU)" annotation(
        Placement(transformation(origin = {35, 29}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, -28}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput qRef "Richiesta di potenza reattiva (PU)" annotation(
        Placement(transformation(origin = {-30, -114}, extent = {{-18, -18}, {18, 18}}), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));
      Park.AbcToDq0 toVdq0 annotation(
        Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
      Park.AbcToDq0 toIdq0 annotation(
        Placement(transformation(origin = {-48, 38}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Routing.Multiplex mux(n = 6) annotation(
        Placement(transformation(extent = {{-24, 40}, {-4, 60}})));
      Blocks.P_f P_f1 annotation(
        Placement(transformation(extent = {{-8, -8}, {8, 8}}, rotation = 0, origin = {44, 60})));
      Blocks.Q_f Q_f1 annotation(
        Placement(transformation(extent = {{-8, -8}, {8, 8}}, rotation = 0, origin = {26, -88})));
      Modelica.Blocks.Interfaces.RealOutput theta "Angolo delle fem generate (rad)" annotation(
        Placement(transformation(origin = {308, 38}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {110, -52}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(transformation(extent = {{106, 46}, {124, 64}})));
      Modelica.Blocks.Math.Gain gain1(k = pGain) annotation(
        Placement(transformation(extent = {{144, 44}, {166, 66}})));
      Modelica.Blocks.Sources.Constant fNomPu(k = 1) annotation(
        Placement(transformation(extent = {{138, 18}, {154, 34}})));
      Modelica.Blocks.Math.Add add1(k1 = -1) annotation(
        Placement(transformation(extent = {{180, 28}, {200, 48}})));
      Modelica.Blocks.Math.Gain gain2(k = 2*pi*fNom) annotation(
        Placement(transformation(extent = {{216, 28}, {236, 48}})));
      Modelica.Blocks.Math.Add add2(k2 = -1) annotation(
        Placement(transformation(extent = {{84, -104}, {104, -84}})));
      Modelica.Blocks.Sources.Constant vNomPu(k = 1) annotation(
        Placement(transformation(extent = {{118, -134}, {134, -118}})));
      Modelica.Blocks.Math.Gain gain3(k = qGain) annotation(
        Placement(transformation(extent = {{114, -106}, {138, -82}})));
      Modelica.Blocks.Math.Add add3(k1 = -1) annotation(
        Placement(transformation(extent = {{154, -110}, {174, -90}})));
      Modelica.Blocks.Math.Gain toNewPu(k = uAcNom*sqrt(2/3)/(uDc/2)) "Fa in modo che con 1 pu e tensione DC nominale si generino fem pari alla tensione nominale" annotation(
        Placement(transformation(extent = {{240, -118}, {260, -96}})));
      Modelica.Blocks.Logical.GreaterEqualThreshold gtTwoPi(threshold = 2*pi) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {270, -10})));
      Modelica.Blocks.Sources.Constant theta0(k = 0) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {274, 84})));
      Modelica.Blocks.Continuous.Integrator integrator(use_reset = true, use_set = true, y_start = pi*4/3) annotation(
        Placement(transformation(extent = {{258, 28}, {278, 48}})));
      Modelica.Blocks.Sources.RealExpression realExpression(y = theta) annotation(
        Placement(transformation(origin = {-100, 12}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput mdV "Direct-axis m in V" annotation(
        Placement(transformation(origin = {309, -111}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Math.Product product1 annotation(
        Placement(transformation(extent = {{270, -122}, {290, -102}})));
      Modelica.Blocks.Math.Gain half(k = 0.5) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {260, -168})));
      Modelica.Blocks.Interfaces.RealInput uDcMeas annotation(
        Placement(transformation(origin = {260, -226}, extent = {{-18, -18}, {18, 18}}, rotation = 90), iconTransformation(origin = {20, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
      Modelica.Blocks.Math.Gain toVpu[3](k = (1/(uAcNom/sqrt(3)*sqrt(2)))*ones(3)) annotation(
        Placement(transformation(origin = {-88, 74}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Math.Gain toIpu[3](k = (1/(sNom/sqrt(3)/uAcNom*sqrt(2)))*ones(3)) annotation(
        Placement(transformation(origin = {-91, 41}, extent = {{-9, -9}, {9, 9}})));
      Modelica.Blocks.Continuous.FirstOrder PfirstOrder(T = fo) annotation(
        Placement(transformation(extent = {{70, 52}, {86, 68}})));
      Modelica.Blocks.Continuous.FirstOrder QfirstOrder(T = fo) annotation(
        Placement(transformation(extent = {{50, -98}, {70, -78}})));
      Modelica.Blocks.Math.Add add4(k2 = -1) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 0, origin = {81, 25})));
      Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = 1/pGain/1, outMax = 1, use_reset = true) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {44, -4})));
      Modelica.Blocks.Math.Feedback feedback annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {178, -48})));
      Modelica.Blocks.Nonlinear.DeadZone deadZone(uMax = secRegBand) annotation(
        Placement(transformation(extent = {{96, -58}, {76, -38}})));
      Modelica.Blocks.Logical.And mode "false=island" annotation(
        Placement(transformation(extent = {{130, -12}, {118, 0}})));
      Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold = 1 - secRegBand) annotation(
        Placement(transformation(extent = {{158, -24}, {146, -12}})));
      Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold = 1 + secRegBand) annotation(
        Placement(transformation(extent = {{158, -4}, {146, 8}})));
      Modelica.Blocks.Math.Gain toHz(k = 50) annotation(
        Placement(transformation(extent = {{212, 70}, {232, 90}})));
      Modelica.Blocks.Math.Feedback feedback1 annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {146, -192})));
      Modelica.Blocks.Continuous.LimIntegrator limIntegrator1(k = 1/qGain/1, outMax = 1, use_reset = true) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {10, -140})));
      Modelica.Blocks.Math.Add add5(k2 = -1) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 0, origin = {49, -119})));
      Modelica.Blocks.Sources.RealExpression Qcalc(y = Q_f1.y) annotation(
        Placement(transformation(extent = {{232, -220}, {212, -200}})));
      Modelica.Blocks.Math.Gain qx(k = xcc) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {204, -180})));
      Modelica.Blocks.Math.Add add6 annotation(
        Placement(transformation(extent = {{210, -116}, {230, -96}})));
      Modelica.Blocks.Continuous.FirstOrder QxfirstOrder(T = fo) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {204, -144})));
      Modelica.Blocks.Sources.BooleanExpression onDelay1(y = onDelay.y) annotation(
        Placement(transformation(extent = {{54, -170}, {34, -150}})));
      Modelica.Blocks.Nonlinear.DeadZone deadZone1(uMax = secRegBand) annotation(
        Placement(transformation(extent = {{52, -202}, {32, -182}})));
      Modelica.Blocks.Math.Gain gain4(k = secReg) annotation(
        Placement(transformation(extent = {{56, -58}, {36, -38}})));
      Modelica.Blocks.Math.Gain gain5(k = secReg) annotation(
        Placement(transformation(extent = {{16, -202}, {-4, -182}})));
      Modelica.Blocks.MathBoolean.OnDelay onDelay(delayTime = secRegResetDelay) annotation(
        Placement(transformation(extent = {{100, -22}, {88, -10}})));
    equation
      connect(toVdq0.dq0, mux.u[1:3]) annotation(
        Line(points = {{-39.8, 70}, {-32, 70}, {-32, 49.4167}, {-24, 49.4167}}, color = {0, 0, 127}));
      connect(toIdq0.dq0, mux.u[4:6]) annotation(
        Line(points = {{-37.8, 38}, {-30, 38}, {-30, 52.9167}, {-24, 52.9167}}, color = {0, 0, 127}));
      connect(add.y, gain1.u) annotation(
        Line(points = {{124.9, 55}, {141.8, 55}}, color = {0, 0, 127}));
      connect(gain1.y, add1.u1) annotation(
        Line(points = {{167.1, 55}, {172, 55}, {172, 44}, {178, 44}}, color = {0, 0, 127}));
      connect(fNomPu.y, add1.u2) annotation(
        Line(points = {{154.8, 26}, {172, 26}, {172, 32}, {178, 32}}, color = {0, 0, 127}));
      connect(add1.y, gain2.u) annotation(
        Line(points = {{201, 38}, {214, 38}}, color = {0, 0, 127}));
      connect(add2.y, gain3.u) annotation(
        Line(points = {{105, -94}, {111.6, -94}}, color = {0, 0, 127}));
      connect(gain3.y, add3.u1) annotation(
        Line(points = {{139.2, -94}, {152, -94}}, color = {0, 0, 127}));
      connect(vNomPu.y, add3.u2) annotation(
        Line(points = {{134.8, -126}, {146, -126}, {146, -106}, {152, -106}}, color = {0, 0, 127}));
      connect(mux.y, P_f1.u) annotation(
        Line(points = {{-3, 50}, {24, 50}, {24, 61.12}, {35.68, 61.12}}, color = {0, 0, 127}));
      connect(Q_f1.u, mux.y) annotation(
        Line(points = {{17.68, -86.88}, {10, -86.88}, {10, 50}, {-3, 50}}, color = {0, 0, 127}));
      connect(theta0.y, integrator.set) annotation(
        Line(points = {{274, 73}, {274, 50}}, color = {0, 0, 127}));
      connect(gtTwoPi.y, integrator.reset) annotation(
        Line(points = {{259, -10}, {254, -10}, {254, 18}, {274, 18}, {274, 26}}, color = {255, 0, 255}));
      connect(integrator.y, theta) annotation(
        Line(points = {{279, 38}, {308, 38}}, color = {0, 0, 127}));
      connect(gtTwoPi.u, integrator.y) annotation(
        Line(points = {{282, -10}, {290, -10}, {290, 38}, {279, 38}}, color = {0, 0, 127}));
      connect(mdV, product1.y) annotation(
        Line(points = {{309, -111}, {309, -112}, {291, -112}}, color = {0, 0, 127}));
      connect(half.u, uDcMeas) annotation(
        Line(points = {{260, -180}, {260, -226}}, color = {0, 0, 127}));
      connect(product1.u2, half.y) annotation(
        Line(points = {{268, -118}, {260, -118}, {260, -157}}, color = {0, 0, 127}));
      connect(product1.u1, toNewPu.y) annotation(
        Line(points = {{268, -106}, {268, -107}, {261, -107}}, color = {0, 0, 127}));
      connect(toIdq0.wt, realExpression.y) annotation(
        Line(points = {{-57.8, 33.2}, {-68, 33.2}, {-68, 12}, {-89, 12}}, color = {0, 0, 127}));
      connect(toVdq0.wt, realExpression.y) annotation(
        Line(points = {{-59.8, 65.2}, {-68, 65.2}, {-68, 12}, {-89, 12}}, color = {0, 0, 127}));
      connect(toVpu.u, vabc) annotation(
        Line(points = {{-100, 74}, {-102, 73}, {-129, 73}}, color = {0, 0, 127}));
      connect(toVdq0.abc, toVpu.y) annotation(
        Line(points = {{-60, 73.4}, {-60, 74}, {-77, 74}}, color = {0, 0, 127}));
      connect(toIdq0.abc, toIpu.y) annotation(
        Line(points = {{-58, 41.4}, {-60, 41}, {-81.1, 41}}, color = {0, 0, 127}));
      connect(toIpu.u, iabc) annotation(
        Line(points = {{-101.8, 41}, {-129, 41}}, color = {0, 0, 127}));
      connect(PfirstOrder.y, add.u1) annotation(
        Line(points = {{86.8, 60}, {96, 60}, {96, 60.4}, {104.2, 60.4}}, color = {0, 0, 127}));
      connect(QfirstOrder.y, add2.u1) annotation(
        Line(points = {{71, -88}, {82, -88}}, color = {0, 0, 127}));
      connect(P_f1.y, PfirstOrder.u) annotation(
        Line(points = {{52.48, 60.16}, {58, 60.16}, {58, 60}, {68.4, 60}}, color = {0, 0, 127}));
      connect(Q_f1.y, QfirstOrder.u) annotation(
        Line(points = {{34.8, -88}, {48, -88}}, color = {0, 0, 127}));
      connect(feedback.u1, add1.y) annotation(
        Line(points = {{186, -48}, {206, -48}, {206, 38}, {201, 38}}, color = {0, 0, 127}));
      connect(feedback.u2, fNomPu.y) annotation(
        Line(points = {{178, -40}, {178, 26}, {154.8, 26}}, color = {0, 0, 127}));
      connect(mdV, mdV) annotation(
        Line(points = {{309, -111}, {309, -111}}, color = {0, 0, 127}));
      connect(lessThreshold.y, mode.u1) annotation(
        Line(points = {{145.4, 2}, {138, 2}, {138, -6}, {131.2, -6}}, color = {255, 0, 255}));
      connect(lessThreshold.u, add1.y) annotation(
        Line(points = {{159.2, 2}, {206, 2}, {206, 38}, {201, 38}}, color = {0, 0, 127}));
      connect(greaterThreshold.u, add1.y) annotation(
        Line(points = {{159.2, -18}, {166, -18}, {166, 2}, {206, 2}, {206, 38}, {201, 38}}, color = {0, 0, 127}));
      connect(pRef, add4.u1) annotation(
        Line(points = {{35, 29}, {68, 29}, {68, 29.2}, {72.6, 29.2}}, color = {0, 0, 127}));
      connect(toHz.u, add1.y) annotation(
        Line(points = {{210, 80}, {202, 80}, {202, 54}, {206, 54}, {206, 38}, {201, 38}}, color = {0, 0, 127}));
      connect(limIntegrator.y, add4.u2) annotation(
        Line(points = {{55, -4}, {68, -4}, {68, 20.8}, {72.6, 20.8}}, color = {0, 0, 127}));
      connect(add4.y, add.u2) annotation(
        Line(points = {{88.7, 25}, {98, 25}, {98, 49.6}, {104.2, 49.6}}, color = {0, 0, 127}));
      connect(feedback1.u2, add3.u2) annotation(
        Line(points = {{146, -184}, {146, -106}, {152, -106}}, color = {0, 0, 127}));
      connect(feedback1.u1, add3.y) annotation(
        Line(points = {{154, -192}, {172, -192}, {172, -150}, {186, -150}, {186, -100}, {175, -100}}, color = {0, 0, 127}));
      connect(limIntegrator1.y, add5.u2) annotation(
        Line(points = {{21, -140}, {34, -140}, {34, -123.2}, {40.6, -123.2}}, color = {0, 0, 127}));
      connect(qRef, add5.u1) annotation(
        Line(points = {{-30, -114}, {-30, -114.8}, {40.6, -114.8}}, color = {0, 0, 127}));
      connect(add5.y, add2.u2) annotation(
        Line(points = {{56.7, -119}, {56.7, -120}, {82, -120}, {82, -100}}, color = {0, 0, 127}));
      connect(gain2.y, integrator.u) annotation(
        Line(points = {{237, 38}, {256, 38}}, color = {0, 0, 127}));
      connect(qx.u, Qcalc.y) annotation(
        Line(points = {{204, -192}, {204, -210}, {211, -210}}, color = {0, 0, 127}));
      connect(add3.y, add6.u1) annotation(
        Line(points = {{175, -100}, {208, -100}}, color = {0, 0, 127}));
      connect(toNewPu.u, add6.y) annotation(
        Line(points = {{238, -107}, {238, -106}, {231, -106}}, color = {0, 0, 127}));
      connect(qx.y, QxfirstOrder.u) annotation(
        Line(points = {{204, -169}, {204, -156}}, color = {0, 0, 127}));
      connect(QxfirstOrder.y, add6.u2) annotation(
        Line(points = {{204, -133}, {204, -112}, {208, -112}}, color = {0, 0, 127}));
      connect(greaterThreshold.y, mode.u2) annotation(
        Line(points = {{145.4, -18}, {138, -18}, {138, -10.8}, {131.2, -10.8}}, color = {255, 0, 255}));
      connect(deadZone.y, gain4.u) annotation(
        Line(points = {{75, -48}, {58, -48}}, color = {0, 0, 127}));
      connect(gain4.y, limIntegrator.u) annotation(
        Line(points = {{35, -48}, {22, -48}, {22, -4}, {32, -4}}, color = {0, 0, 127}));
      connect(deadZone1.y, gain5.u) annotation(
        Line(points = {{31, -192}, {18, -192}}, color = {0, 0, 127}));
      connect(gain5.y, limIntegrator1.u) annotation(
        Line(points = {{-5, -192}, {-16, -192}, {-16, -140}, {-2, -140}}, color = {0, 0, 127}));
      connect(onDelay.u, mode.y) annotation(
        Line(points = {{102.4, -16}, {112, -16}, {112, -6}, {117.4, -6}}, color = {255, 0, 255}));
      connect(feedback.y, deadZone.u) annotation(
        Line(points = {{169, -48}, {98, -48}}, color = {0, 0, 127}));
      connect(feedback1.y, deadZone1.u) annotation(
        Line(points = {{137, -192}, {54, -192}}, color = {0, 0, 127}));
      connect(onDelay.y, limIntegrator.reset) annotation(
        Line(points = {{86.8, -16}, {74, -16}, {74, -24}, {50, -24}, {50, -16}}, color = {255, 0, 255}));
      connect(onDelay1.y, limIntegrator1.reset) annotation(
        Line(points = {{33, -160}, {16, -160}, {16, -152}}, color = {255, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-120, -220}, {300, 100}})),
        Icon(graphics = {Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-122, 132}, {110, 106}}, textString = "%name"), Text(textColor = {238, 46, 47}, extent = {{-70, 34}, {90, -30}}, textString = "GFD
droop
P-f Q-v"), Text(extent = {{56, -46}, {96, -60}}, textString = "q", fontName = "Symbol", textColor = {0, 0, 0}, textStyle = {TextStyle.Italic}), Text(extent = {{-102, 86}, {-52, 72}}, textString = "v", textColor = {0, 0, 0}), Text(extent = {{-102, 36}, {-52, 22}}, textString = "i", textColor = {0, 0, 0}), Text(extent = {{-96, -20}, {-46, -34}}, textString = "pRef", textColor = {0, 0, 0}), Text(extent = {{-96, -72}, {-46, -86}}, textColor = {0, 0, 0}, textString = "qRef"), Text(extent = {{46, 68}, {100, 54}}, textColor = {0, 0, 0}, textString = "mdV"), Text(extent = {{-8, -78}, {46, -92}}, textColor = {0, 0, 0}, textString = "uDc")}));
    end GfmDroopControl;

    model Gflcontrol
      import Modelica.Constants.pi;
      parameter Modelica.Units.SI.Frequency fNom = 50 "Nominal frequency";
      parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Nominal apparent power";
      parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph line-to-line voltage (RMS), used for per-unit";
      parameter Modelica.Units.SI.Resistance rf = 0.00314 "Thevenin (circa filter) resistance ";
      parameter Modelica.Units.SI.Inductance lf = 5e-4 "Thevenin (circa filter) inductance ";
      parameter Modelica.Units.SI.Capacitance cf = 6e-6 "Filter capacitance";
      parameter Modelica.Units.SI.Time delaycontrol = 0.05 "GFL control id iq first order";
      parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom "Nominal Impedance";
      parameter Modelica.Units.SI.Reactance x = 2*pi*fNom*lf/zNom "VSC Reactance (PU)";
      parameter Modelica.Units.SI.Time tau_i = 1/(4*2*pi*fNom) "Time constant of current loop";
      parameter Modelica.Units.SI.Time tau_v = 1/(4*2*pi*fNom) "Time constant of feedforward vd e vq";
      parameter Real kp_i = x/tau_i/100/pi "Proportional gain of PI current regulators";
      parameter Real ki_i = kp_i/(0.05*100*pi) "Integral gain of PI current regulators";
      Modelica.Blocks.Interfaces.RealInput vabc[3] annotation(
        Placement(transformation(origin = {-168, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput iabc[3] annotation(
        Placement(transformation(origin = {-166, 46}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput theta annotation(
        Placement(transformation(origin = {-166, 8}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput pref annotation(
        Placement(transformation(origin = {-166, -22}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput qref annotation(
        Placement(transformation(origin = {-166, -68}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));
      Park.AbcToDq0 vToDq0 annotation(
        Placement(transformation(origin = {-64, 80}, extent = {{-10, -10}, {10, 10}})));
      Park.AbcToDq0 iToDq0 annotation(
        Placement(transformation(origin = {-12, 18}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Math.Gain gain(k = -1) annotation(
        Placement(transformation(origin = {-12, 0}, extent = {{-68, -78}, {-48, -58}})));
      AwuFirstOrder awu_vd(tau_v = tau_v) annotation(
        Placement(transformation(origin = {-48, 0}, extent = {{24, 70}, {44, 90}})));
      AwuFirstOrder awu_vq(tau_v = tau_v) annotation(
        Placement(transformation(origin = {-36, 0}, extent = {{24, -114}, {44, -94}})));
      Modelica.Blocks.Math.Add3 add3 annotation(
        Placement(transformation(origin = {-56, 0}, extent = {{202, -78}, {222, -58}})));
      Modelica.Blocks.Math.Gain gain1(k = x) annotation(
        Placement(transformation(origin = {-46, 0}, extent = {{126, -2}, {146, 18}})));
      Modelica.Blocks.Math.Gain gain2(k = x) annotation(
        Placement(transformation(origin = {-46, 0}, extent = {{126, -42}, {146, -22}})));
      Modelica.Blocks.Math.Add3 add2(k3 = -1) annotation(
        Placement(transformation(origin = {-50, 0}, extent = {{194, 46}, {214, 66}})));
      Modelica.Blocks.Continuous.LimPID PI_id(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = kp_i, Ti = ki_i, yMax = 1) annotation(
        Placement(transformation(origin = {-56, -2}, extent = {{128, 46}, {148, 66}})));
      Modelica.Blocks.Continuous.LimPID PI_iq(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = kp_i, Ti = ki_i, yMax = 1) annotation(
        Placement(transformation(origin = {-48, 0}, extent = {{128, -78}, {148, -58}})));
      Modelica.Blocks.Interfaces.RealOutput mdqV[2] annotation(
        Placement(transformation(origin = {300, -2}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder(T = delaycontrol) annotation(
        Placement(transformation(origin = {-40, 0}, extent = {{68, 22}, {88, 42}})));
      Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = delaycontrol) annotation(
        Placement(transformation(origin = {-44, 0}, extent = {{72, -42}, {92, -22}})));
      Modelica.Blocks.Interfaces.RealInput uDc annotation(
        Placement(transformation(origin = {144, -148}, extent = {{-18, -18}, {18, 18}}, rotation = 90), iconTransformation(origin = {20, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
      Modelica.Blocks.Math.Gain half(k = 0.5) annotation(
        Placement(transformation(extent = {{164, -132}, {184, -110}})));
      Modelica.Blocks.Math.Product product1 annotation(
        Placement(transformation(extent = {{208, 40}, {228, 60}})));
      Modelica.Blocks.Math.Product product2 annotation(
        Placement(transformation(extent = {{214, -84}, {234, -64}})));
      Modelica.Blocks.Math.Gain toVpu[3](k = (1/(uAcNom/sqrt(3)*sqrt(2)))*ones(3)) annotation(
        Placement(transformation(origin = {-116, 80}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Math.Gain toIpu[3](k = (1/(sNom/sqrt(3)/uAcNom*sqrt(2)))*ones(3)) annotation(
        Placement(transformation(origin = {-116, 46}, extent = {{-10, -10}, {10, 10}})));
    equation
      connect(vToDq0.wt, theta) annotation(
        Line(points = {{-73.8, 75.2}, {-84, 75.2}, {-84, 8}, {-166, 8}}, color = {0, 0, 127}));
      connect(iToDq0.wt, theta) annotation(
        Line(points = {{-21.8, 13.2}, {-84, 13.2}, {-84, 8}, {-166, 8}}, color = {0, 0, 127}));
      connect(qref, gain.u) annotation(
        Line(points = {{-166, -68}, {-82, -68}}, color = {0, 0, 127}));
      connect(gain1.y, add3.u1) annotation(
        Line(points = {{101, 8}, {144, 8}, {144, -60}}, color = {0, 0, 127}));
      connect(awu_vq.v_out, add3.u3) annotation(
        Line(points = {{7.6, -104}, {144, -104}, {144, -76}}, color = {0, 0, 127}));
      connect(gain2.y, add2.u3) annotation(
        Line(points = {{101, -32}, {114, -32}, {114, 48}, {142, 48}}, color = {0, 0, 127}));
      connect(awu_vd.v_out, add2.u1) annotation(
        Line(points = {{-4.4, 80}, {142, 80}, {142, 64}}, color = {0, 0, 127}));
      connect(PI_id.u_s, pref) annotation(
        Line(points = {{70, 54}, {-68, 54}, {-68, -22}, {-166, -22}}, color = {0, 0, 127}));
      connect(PI_id.y, add2.u2) annotation(
        Line(points = {{93, 54}, {118, 54}, {118, 56}, {142, 56}}, color = {0, 0, 127}));
      connect(PI_iq.y, add3.u2) annotation(
        Line(points = {{101, -68}, {144, -68}}, color = {0, 0, 127}));
      connect(PI_iq.u_s, gain.y) annotation(
        Line(points = {{78, -68}, {-59, -68}}, color = {0, 0, 127}));
      connect(firstOrder.u, iToDq0.dq0[1]) annotation(
        Line(points = {{26, 32}, {16, 32}, {16, 18}, {14, 18}, {14, 17.6667}, {-1.8, 17.6667}}, color = {0, 0, 127}));
      connect(firstOrder.y, PI_id.u_m) annotation(
        Line(points = {{49, 32}, {82, 32}, {82, 42}}, color = {0, 0, 127}));
      connect(gain1.u, firstOrder.y) annotation(
        Line(points = {{78, 8}, {58, 8}, {58, 32}, {49, 32}}, color = {0, 0, 127}));
      connect(firstOrder1.u, iToDq0.dq0[2]) annotation(
        Line(points = {{26, -32}, {2, -32}, {2, 18}, {-1.8, 18}}, color = {0, 0, 127}));
      connect(gain2.u, firstOrder1.y) annotation(
        Line(points = {{78, -32}, {49, -32}}, color = {0, 0, 127}));
      connect(gain2.u, PI_iq.u_m) annotation(
        Line(points = {{78, -32}, {62, -32}, {62, -92}, {90, -92}, {90, -80}}, color = {0, 0, 127}));
      connect(uDc, half.u) annotation(
        Line(points = {{144, -148}, {144, -121}, {162, -121}}, color = {0, 0, 127}));
      connect(add2.y, product1.u1) annotation(
        Line(points = {{165, 56}, {206, 56}}, color = {0, 0, 127}));
      connect(product1.y, mdqV[1]) annotation(
        Line(points = {{229, 50}, {284, 50}, {284, -4.5}, {300, -4.5}}, color = {0, 0, 127}));
      connect(half.y, product1.u2) annotation(
        Line(points = {{185, -121}, {190, -121}, {190, 44}, {206, 44}}, color = {0, 0, 127}));
      connect(add3.y, product2.u1) annotation(
        Line(points = {{167, -68}, {212, -68}}, color = {0, 0, 127}));
      connect(product2.y, mdqV[2]) annotation(
        Line(points = {{235, -74}, {284, -74}, {284, 0.5}, {300, 0.5}}, color = {0, 0, 127}));
      connect(product2.u2, half.y) annotation(
        Line(points = {{212, -80}, {190, -80}, {190, -121}, {185, -121}}, color = {0, 0, 127}));
      connect(toVpu.u, vabc) annotation(
        Line(points = {{-128, 80}, {-168, 80}}, color = {0, 0, 127}));
      connect(toVpu.y, vToDq0.abc) annotation(
        Line(points = {{-105, 80}, {-84, 80}, {-84, 83.4}, {-74, 83.4}}, color = {0, 0, 127}));
      connect(toIpu.u, iabc) annotation(
        Line(points = {{-128, 46}, {-166, 46}}, color = {0, 0, 127}));
      connect(toIpu.y, iToDq0.abc) annotation(
        Line(points = {{-105, 46}, {-32, 46}, {-32, 21.4}, {-22, 21.4}}, color = {0, 0, 127}));
      connect(vToDq0.dq0[1], awu_vd.v_in) annotation(
        Line(points = {{-53.8, 79.6667}, {-52, 80}, {-24.2, 80}}, color = {0, 0, 127}));
      connect(vToDq0.dq0[2], awu_vq.v_in) annotation(
        Line(points = {{-53.8, 80}, {-48, 80}, {-48, -104}, {-12.2, -104}}, color = {0, 0, 127}));
      annotation(
        Diagram(coordinateSystem(extent = {{-160, -140}, {320, 100}}), graphics = {Text(origin = {-46, 0}, textColor = {238, 46, 47}, extent = {{162, -26}, {176, -32}}, textString = "x*iq"), Text(origin = {-46, 0}, textColor = {238, 46, 47}, extent = {{172, 14}, {186, 8}}, textString = "x*id"), Text(origin = {-60, 22}, textColor = {238, 46, 47}, extent = {{228, 30}, {242, 24}}, textString = "md"), Text(origin = {-50, -28}, textColor = {238, 46, 47}, extent = {{218, -32}, {232, -38}}, textString = "mq"), Text(origin = {-214, 74}, textColor = {238, 46, 47}, extent = {{172, 14}, {186, 8}}, textString = "vd"), Text(origin = {-425.71, 64.6666}, textColor = {238, 46, 47}, extent = {{417.71, 9.3334}, {451.71, 5.33338}}, textString = "F(s)_ieee"), Text(origin = {-409.71, -105.333}, textColor = {238, 46, 47}, extent = {{417.71, 9.3334}, {451.71, 5.33338}}, textString = "F(s)_ieee"), Text(origin = {-204, -110}, textColor = {238, 46, 47}, extent = {{172, 14}, {186, 8}}, textString = "vq"), Text(origin = {-327.71, 62.6666}, textColor = {238, 46, 47}, extent = {{417.71, 9.3334}, {451.71, 5.33338}}, textString = "Ki(s)_ieee"), Text(origin = {-327.71, -91.333}, textColor = {238, 46, 47}, extent = {{417.71, 9.3334}, {451.71, 5.33338}}, textString = "Ki(s)_ieee"), Text(origin = {-406.281, 42.6666}, textColor = {238, 46, 47}, extent = {{442.281, 23.3334}, {478.281, 13.3333}}, textString = "~id_ref_ieee"), Text(origin = {-401.71, -76.6668}, textColor = {238, 46, 47}, extent = {{417.71, 18.6668}, {451.71, 10.6667}}, textString = "~iq_ref_ieee")}),
        Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, rotation = 90), Text(extent = {{-100, 128}, {100, 100}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-60, 22}, {74, -20}}, textColor = {238, 46, 47}, textString = "GFL
control"), Text(extent = {{-94, 88}, {-44, 74}}, textString = "v", textColor = {0, 0, 0}), Text(extent = {{-94, 48}, {-44, 34}}, textColor = {0, 0, 0}, textString = "i"), Text(extent = {{-90, 6}, {-40, -8}}, textColor = {0, 0, 0}, textString = "pRef"), Text(extent = {{-86, -36}, {-36, -50}}, textColor = {0, 0, 0}, textString = "qRref"), Text(extent = {{-88, -72}, {-48, -86}}, textString = "q", fontName = "Symbol", textColor = {0, 0, 0}, textStyle = {TextStyle.Italic}), Text(extent = {{4, -78}, {58, -92}}, textColor = {0, 0, 0}, textString = "uDc"), Text(extent = {{54, 8}, {104, -6}}, textColor = {0, 0, 0}, textString = "mdqV")}));
    end Gflcontrol;

    package Vsm
      model PowerCalcVsm
        Modelica.Blocks.Interfaces.RealInput vdq0[3] annotation(
          Placement(transformation(origin = {-109, 49}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, 60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput idq0[3] annotation(
          Placement(transformation(origin = {-109, 1}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput ivdq[2] annotation(
          Placement(transformation(origin = {-109, -47}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, -60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput pi annotation(
          Placement(transformation(origin = {103, 53}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, 54}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput pv annotation(
          Placement(transformation(origin = {103, 15}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, 16}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput qi annotation(
          Placement(transformation(origin = {103, -13}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, -22}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput qv annotation(
          Placement(transformation(origin = {103, -45}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, -58}, extent = {{-10, -10}, {10, 10}})));
      equation
        pv = vdq0[1]*ivdq[1] + vdq0[2]*ivdq[2];
        qv = vdq0[2]*ivdq[1] - vdq0[1]*ivdq[2];
        pi = vdq0[1]*idq0[1] + vdq0[2]*idq0[2];
        qi = vdq0[2]*idq0[1] - vdq0[1]*idq0[2];
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-120, 128}, {112, 102}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-92, 68}, {-42, 54}}, textColor = {0, 0, 0}, textString = "vdq0"), Text(extent = {{-92, 8}, {-42, -6}}, textColor = {0, 0, 0}, textString = "idq0"), Text(extent = {{-92, -54}, {-42, -68}}, textColor = {0, 0, 0}, textString = "ivdq"), Text(extent = {{46, 62}, {96, 48}}, textColor = {0, 0, 0}, textString = "Pi"), Text(extent = {{46, 22}, {96, 8}}, textColor = {0, 0, 0}, textString = "Pv"), Text(extent = {{46, -16}, {96, -30}}, textColor = {0, 0, 0}, textString = "Qi"), Text(extent = {{50, -52}, {100, -66}}, textColor = {0, 0, 0}, textString = "Qv")}),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(extent = {{-36, 30}, {48, -32}}, textColor = {0, 0, 0}, horizontalAlignment = TextAlignment.Left, textString = "vsm:
Pv = vd*ivd + vq*ivq
Qv = vq*ivd − vd*ivq

total inverter output:
Pi = vd*id + vq*iq
Qi = vq*id − vd*iq")}));
      end PowerCalcVsm;

      block ActiveLoopVsm
        import Modelica.Constants.pi;
        import Modelica.Constants.inf;
        parameter Modelica.Units.SI.Frequency fNom = 50 "Frequenza nominale";
        parameter Modelica.Units.SI.Time h = 4 "costante inerzia";
        parameter Real ki_al = 1/(2*h) "ki PI parallelo, qui solo integratore, ovvero Kh in figura 7.3 tesi";
        Modelica.Blocks.Interfaces.RealInput pvRef "Richiesta di potenza attiva dal sincrono virtuale (PU)" annotation(
          Placement(transformation(origin = {-107, 41}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-112, 44}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput pv annotation(
          Placement(transformation(origin = {-105, -37}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, -48}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Continuous.Integrator integrator(k = 2*pi*fNom, use_reset = true, use_set = true, initType = Modelica.Blocks.Types.Init.NoInit, y_start = pi*1) annotation(
          Placement(transformation(extent = {{28, -34}, {48, -14}})));
        Modelica.Blocks.Logical.GreaterEqualThreshold gtTwoPi(threshold = 2*pi) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {62, -70})));
        Modelica.Blocks.Sources.Constant theta0(k = 0) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {44, 20})));
        Modelica.Blocks.Interfaces.RealOutput w "VSG rotational speed (PU)" annotation(
          Placement(transformation(origin = {107, 59}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, 48}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput theta "rad" annotation(
          Placement(transformation(origin = {107, -39}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, -46}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Continuous.Integrator integrator1(k = ki_al, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 1) annotation(
          Placement(transformation(extent = {{-10, 30}, {10, 50}})));
        Modelica.Blocks.Math.Add add(k2 = -1) annotation(
          Placement(transformation(extent = {{-48, 30}, {-28, 50}})));
        Modelica.Blocks.Math.Gain toHz(k = 50) annotation(
          Placement(transformation(extent = {{40, 74}, {60, 94}})));
      equation
        connect(gtTwoPi.y, integrator.reset) annotation(
          Line(points = {{51, -70}, {44, -70}, {44, -36}}, color = {255, 0, 255}));
        connect(theta0.y, integrator.set) annotation(
          Line(points = {{44, 9}, {44, -12}}, color = {0, 0, 127}));
        connect(integrator.y, theta) annotation(
          Line(points = {{49, -24}, {84, -24}, {84, -39}, {107, -39}}, color = {0, 0, 127}));
        connect(integrator.y, gtTwoPi.u) annotation(
          Line(points = {{49, -24}, {84, -24}, {84, -70}, {74, -70}}, color = {0, 0, 127}));
        connect(theta, theta) annotation(
          Line(points = {{107, -39}, {107, -39}}, color = {0, 0, 127}));
        connect(pv, add.u2) annotation(
          Line(points = {{-105, -37}, {-58, -37}, {-58, 34}, {-50, 34}}, color = {0, 0, 127}));
        connect(pvRef, add.u1) annotation(
          Line(points = {{-107, 41}, {-58, 41}, {-58, 46}, {-50, 46}}, color = {0, 0, 127}));
        connect(add.y, integrator1.u) annotation(
          Line(points = {{-27, 40}, {-12, 40}}, color = {0, 0, 127}));
        connect(integrator1.y, w) annotation(
          Line(points = {{11, 40}, {90, 40}, {90, 59}, {107, 59}}, color = {0, 0, 127}));
        connect(integrator1.y, integrator.u) annotation(
          Line(points = {{11, 40}, {16, 40}, {16, -24}, {26, -24}}, color = {0, 0, 127}));
        connect(toHz.u, integrator1.y) annotation(
          Line(points = {{38, 84}, {22, 84}, {22, 40}, {11, 40}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-118, 128}, {114, 102}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-92, 60}, {-16, 34}}, textColor = {0, 0, 0}, textString = "pvRef"), Text(extent = {{-102, -32}, {-38, -58}}, textColor = {0, 0, 0}, textString = "pv"), Text(extent = {{46, -30}, {88, -52}}, textColor = {0, 0, 0}, fontName = "Symbol", textString = "q", textStyle = {TextStyle.Italic}), Text(extent = {{38, 60}, {96, 42}}, textColor = {0, 0, 0}, textString = "w")}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end ActiveLoopVsm;

      block ReactiveLoopVsm
        import Modelica.Constants.inf;
        parameter Modelica.Units.SI.Inductance lfg = 0 "filter LCL grid side inductance";
        parameter Modelica.Units.SI.Inductance lg = 2.44e-5 "grid side inductance";
        parameter Real lv = 0.1 "induttanza virtuale (PU)";
        parameter Modelica.Units.SI.Time te = 1 "costante tempo eccitazione";
        parameter Real ke = (lv + lg + lfg)/te "guadagno statico integratore, Ke fig.7.4 su tesi";
        Modelica.Blocks.Interfaces.RealInput qvRef "Richiesta di potenza reattiva dal sincrono virtuale (PU)" annotation(
          Placement(transformation(origin = {-107, 41}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-112, 44}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput qv annotation(
          Placement(transformation(origin = {-107, -31}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, -48}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Math.Add add1(k2 = -1) annotation(
          Placement(transformation(extent = {{-44, -10}, {-24, 10}})));
        Modelica.Blocks.Interfaces.RealOutput fe "flusso di eccitazione" annotation(
          Placement(transformation(origin = {110, 0}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = ke, outMax = inf, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 1) annotation(
          Placement(transformation(extent = {{16, -10}, {36, 10}})));
      equation
        connect(qvRef, add1.u1) annotation(
          Line(points = {{-107, 41}, {-54, 41}, {-54, 6}, {-46, 6}}, color = {0, 0, 127}));
        connect(add1.u2, qv) annotation(
          Line(points = {{-46, -6}, {-82, -6}, {-82, -31}, {-107, -31}}, color = {0, 0, 127}));
        connect(fe, fe) annotation(
          Line(points = {{110, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(add1.y, limIntegrator.u) annotation(
          Line(points = {{-23, 0}, {14, 0}}, color = {0, 0, 127}));
        connect(limIntegrator.y, fe) annotation(
          Line(points = {{37, 0}, {110, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-118, 128}, {114, 102}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-90, 64}, {-6, 30}}, textColor = {0, 0, 0}, textString = "qvRef"), Text(extent = {{-102, -34}, {-34, -58}}, textColor = {0, 0, 0}, textString = "qv"), Text(extent = {{28, 14}, {106, -12}}, textColor = {0, 0, 0}, textString = "fe")}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end ReactiveLoopVsm;

      model CurrentRefCalcVsm
        Modelica.Blocks.Interfaces.RealInput vdq0[3] annotation(
          Placement(transformation(origin = {-109, 49}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, 60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput pRef annotation(
          Placement(transformation(origin = {-109, 1}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, 0}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput qRef annotation(
          Placement(transformation(origin = {-109, -47}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-110, -60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput iRefdq[2] annotation(
          Placement(transformation(origin = {107, -1}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}})));
      equation
        iRefdq[1] = (pRef*vdq0[1] + qRef*vdq0[2])/(vdq0[1]*vdq0[1] + vdq0[2]*vdq0[2]);
        iRefdq[2] = (pRef*vdq0[2] - qRef*vdq0[1])/(vdq0[1]*vdq0[1] + vdq0[2]*vdq0[2]);
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-120, 128}, {112, 102}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-76, 70}, {-26, 56}}, textColor = {0, 0, 0}, textString = "vdq0"), Text(extent = {{-84, 12}, {-14, -10}}, textColor = {0, 0, 0}, textString = "pRefDroop"), Text(extent = {{-86, -48}, {-16, -70}}, textColor = {0, 0, 0}, textString = "qRefDroop"), Text(extent = {{30, 6}, {80, -8}}, textColor = {0, 0, 0}, textString = "iRefdq")}),
          Diagram(coordinateSystem(preserveAspectRatio = false), graphics = {Text(extent = {{-44, 48}, {86, 32}}, textColor = {0, 0, 0}, textString = "Irefd = (Pref*Vd + Qref*Vq)/(Vd^2+Vq^2)"), Text(extent = {{-42, -34}, {84, -50}}, textColor = {0, 0, 0}, textString = "Irefq = (Pref*Vq - Qref*Vd)/(Vd^2+Vq^2)")}));
      end CurrentRefCalcVsm;

      block StatorEqVsm
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency fNom = 50 "Frequenza nominale";
        parameter Modelica.Units.SI.AngularVelocity wn = 2*pi*fNom "wn";
        parameter Modelica.Units.SI.Inductance lfg = 0 "filter LCL grid side inductance";
        parameter Modelica.Units.SI.Inductance lg = 2.44e-5 "grid connection inductance (transformer)";
        parameter Real rv = 0.02 "resistenza virtuale (PU)";
        parameter Real lv = 0.1 "induttanza virtuale (PU)";
        parameter Modelica.Units.SI.Time h = 4 "costante inerzia";
        parameter Real csiMec = 0.7 "smorzamento meccanico swing equation";
        parameter Real trq0 = (1 + 2*csiMec)^(3/2)*sqrt(2*h*(lv + lfg + lg)/wn) "costante di tempo a vuoto dell’avvolgimento di smorzamento virtuale";
        parameter Real lrq = 4*csiMec*(1 + csiMec)*(lv + lfg + lg) "induttanza Lrq";
        Modelica.Blocks.Interfaces.RealInput w annotation(
          Placement(transformation(origin = {74, 4}, extent = {{18, -18}, {-18, 18}}), iconTransformation(origin = {-110, -10}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput fe annotation(
          Placement(transformation(origin = {72, 42}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {-108, -78}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput ivdq[2] annotation(
          Placement(transformation(origin = {287, 1}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Gain gain(k = wn) annotation(
          Placement(transformation(extent = {{-24, 44}, {-4, 64}})));
        Modelica.Blocks.Math.Add3 add annotation(
          Placement(transformation(extent = {{-82, 44}, {-62, 64}})));
        Modelica.Blocks.Math.Add3 add1(k1 = -1) annotation(
          Placement(transformation(extent = {{-64, -52}, {-44, -32}})));
        Modelica.Blocks.Math.Gain gain1(k = wn) annotation(
          Placement(transformation(extent = {{-22, -52}, {-2, -32}})));
        Modelica.Blocks.Math.Gain gain2(k = rv) annotation(
          Placement(transformation(extent = {{-8, 74}, {-28, 94}})));
        Modelica.Blocks.Math.Gain gain3(k = rv) annotation(
          Placement(transformation(extent = {{-6, -92}, {-26, -72}})));
        Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialState, y_start = 1) annotation(
          Placement(transformation(extent = {{22, 44}, {42, 64}})));
        Modelica.Blocks.Continuous.Integrator integrator1(initType = Modelica.Blocks.Types.Init.InitialState, y_start = 0) annotation(
          Placement(transformation(extent = {{20, -52}, {40, -32}})));
        Modelica.Blocks.Math.Product product1 annotation(
          Placement(transformation(extent = {{42, 8}, {22, 28}})));
        Modelica.Blocks.Math.Product product2 annotation(
          Placement(transformation(extent = {{42, -20}, {22, 0}})));
        Modelica.Blocks.Math.Gain gain4(k = 1/lv) annotation(
          Placement(transformation(extent = {{216, 38}, {236, 58}})));
        Modelica.Blocks.Math.Gain gain5(k = 1/lv) annotation(
          Placement(transformation(extent = {{238, -50}, {258, -30}})));
        Modelica.Blocks.Math.Add add2(k1 = -1) annotation(
          Placement(transformation(extent = {{100, 38}, {120, 58}})));
        Modelica.Blocks.Math.Add add3(k1 = -1, k2 = -1) annotation(
          Placement(transformation(extent = {{100, -28}, {120, -8}})));
        Modelica.Blocks.Continuous.Integrator integrator2(initType = Modelica.Blocks.Types.Init.InitialState, y_start = 0) annotation(
          Placement(transformation(extent = {{168, -28}, {188, -8}})));
        Modelica.Blocks.Math.Gain gain6(k = 1/trq0) annotation(
          Placement(transformation(extent = {{138, -28}, {158, -8}})));
        Modelica.Blocks.Math.Add add4(k2 = -1) annotation(
          Placement(transformation(extent = {{208, -50}, {228, -30}})));
        Modelica.Blocks.Math.Gain gain7(k = lrq) annotation(
          Placement(transformation(extent = {{176, 8}, {156, 28}})));
        Modelica.Blocks.Interfaces.RealInput vdq0[3] annotation(
          Placement(transformation(extent = {{-130, 40}, {-90, 80}}), iconTransformation(extent = {{-130, 40}, {-90, 80}})));
      equation
        connect(ivdq, ivdq) annotation(
          Line(points = {{287, 1}, {287, 1}}, color = {0, 0, 127}));
        connect(gain.y, integrator.u) annotation(
          Line(points = {{-3, 54}, {20, 54}}, color = {0, 0, 127}));
        connect(gain1.y, integrator1.u) annotation(
          Line(points = {{-1, -42}, {18, -42}}, color = {0, 0, 127}));
        connect(product1.u1, integrator.y) annotation(
          Line(points = {{44, 24}, {52, 24}, {52, 54}, {43, 54}}, color = {0, 0, 127}));
        connect(product2.u2, integrator1.y) annotation(
          Line(points = {{44, -16}, {52, -16}, {52, -42}, {41, -42}}, color = {0, 0, 127}));
        connect(add.y, gain.u) annotation(
          Line(points = {{-61, 54}, {-26, 54}}, color = {0, 0, 127}));
        connect(add1.y, gain1.u) annotation(
          Line(points = {{-43, -42}, {-24, -42}}, color = {0, 0, 127}));
        connect(product1.y, add1.u1) annotation(
          Line(points = {{21, 18}, {-66, 18}, {-66, -34}}, color = {0, 0, 127}));
        connect(product2.y, add.u3) annotation(
          Line(points = {{21, -10}, {-84, -10}, {-84, 46}}, color = {0, 0, 127}));
        connect(product1.u2, w) annotation(
          Line(points = {{44, 12}, {74, 12}, {74, 4}}, color = {0, 0, 127}));
        connect(product2.u1, w) annotation(
          Line(points = {{44, -4}, {74, -4}, {74, 4}}, color = {0, 0, 127}));
        connect(gain3.y, add1.u3) annotation(
          Line(points = {{-27, -82}, {-66, -82}, {-66, -50}}, color = {0, 0, 127}));
        connect(gain2.y, add.u1) annotation(
          Line(points = {{-29, 84}, {-84, 84}, {-84, 62}}, color = {0, 0, 127}));
        connect(add2.u1, integrator.y) annotation(
          Line(points = {{98, 54}, {43, 54}}, color = {0, 0, 127}));
        connect(add2.u2, fe) annotation(
          Line(points = {{98, 42}, {72, 42}}, color = {0, 0, 127}));
        connect(add2.y, gain4.u) annotation(
          Line(points = {{121, 48}, {214, 48}}, color = {0, 0, 127}));
        connect(gain4.y, ivdq[1]) annotation(
          Line(points = {{237, 48}, {268, 48}, {268, 2}, {270, 2}, {270, -1.75}, {287, -1.75}}, color = {0, 0, 127}));
        connect(gain5.y, ivdq[2]) annotation(
          Line(points = {{259, -40}, {264, -40}, {264, 2}, {287, 2}, {287, 3.75}}, color = {0, 0, 127}));
        connect(gain5.y, gain3.u) annotation(
          Line(points = {{259, -40}, {264, -40}, {264, -82}, {-4, -82}}, color = {0, 0, 127}));
        connect(integrator2.u, gain6.y) annotation(
          Line(points = {{166, -18}, {159, -18}}, color = {0, 0, 127}));
        connect(add3.y, gain6.u) annotation(
          Line(points = {{121, -18}, {136, -18}}, color = {0, 0, 127}));
        connect(integrator2.y, add4.u1) annotation(
          Line(points = {{189, -18}, {194, -18}, {194, -34}, {206, -34}}, color = {0, 0, 127}));
        connect(add4.u2, integrator1.y) annotation(
          Line(points = {{206, -46}, {50, -46}, {50, -42}, {41, -42}}, color = {0, 0, 127}));
        connect(add4.y, gain5.u) annotation(
          Line(points = {{229, -40}, {236, -40}}, color = {0, 0, 127}));
        connect(gain7.y, add3.u1) annotation(
          Line(points = {{155, 18}, {88, 18}, {88, -12}, {98, -12}}, color = {0, 0, 127}));
        connect(gain7.u, gain5.y) annotation(
          Line(points = {{178, 18}, {264, 18}, {264, -40}, {259, -40}}, color = {0, 0, 127}));
        connect(add3.u2, integrator2.y) annotation(
          Line(points = {{98, -24}, {90, -24}, {90, -36}, {194, -36}, {194, -18}, {189, -18}}, color = {0, 0, 127}));
        connect(gain4.y, gain2.u) annotation(
          Line(points = {{237, 48}, {242, 48}, {242, 84}, {-6, 84}}, color = {0, 0, 127}));
        connect(add.u2, vdq0[1]) annotation(
          Line(points = {{-84, 54}, {-84, 53.3333}, {-110, 53.3333}}, color = {0, 0, 127}));
        connect(add1.u2, vdq0[2]) annotation(
          Line(points = {{-66, -42}, {-92, -42}, {-92, 60}, {-110, 60}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(textColor = {0, 0, 255}, extent = {{-118, 128}, {114, 102}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-88, -60}, {-22, -88}}, textColor = {0, 0, 0}, textString = "fe"), Text(extent = {{-84, 78}, {-22, 50}}, textColor = {0, 0, 0}, textString = "vdq0"), Text(extent = {{-94, 12}, {-26, -16}}, textColor = {0, 0, 0}, textString = "w"), Text(extent = {{26, 18}, {92, -10}}, textColor = {0, 0, 0}, textString = "ivdq")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {280, 100}}), graphics = {Text(extent = {{22, 100}, {156, 86}}, textColor = {238, 46, 47}, textString = "da valutare se mettere integratori con limitazione")}));
      end StatorEqVsm;

      block CurrentControlVsm
        import Modelica.Constants.pi;
        import Modelica.Constants.inf;
        parameter Modelica.Units.SI.Frequency fNom = 50 "Frequenza nominale";
        parameter Modelica.Units.SI.Voltage uAcNom = 400 "Val. nominale tensione AC trifase rms (per PU)";
        parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Potenza nominale";
        parameter Modelica.Units.SI.Frequency fbI = 1000 "banda passante regolatore anello corrente in genere a 1/10 della frequenza di switch";
        parameter Modelica.Units.SI.Frequency fzI = fbI/10 "zero frequency";
        parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom;
        parameter Modelica.Units.SI.Inductance lNom = zNom/(2*pi*fNom) "induttanza base";
        parameter Modelica.Units.SI.Inductance lf = 5e-4 "Thevenin (circa filter) inductance ";
        parameter Real leqI = lf/lNom "inductance PU with 'Polito criteria' ->(L)CL -> Leq=Lth in PU";
        parameter Real kpPI = leqI*fbI/fNom "kp PI parallelo=serie";
        parameter Real kiPI = 2*pi*fzI*kpPI "ki PI parallelo. T del PI serie su modelica= Kp_I/Ki_I";
        parameter Modelica.Units.SI.Frequency fCutd = 1000 "fcut LP filter d";
        parameter Modelica.Units.SI.Frequency fCutq = 1000 "fcut LP filter q";
        Modelica.Blocks.Interfaces.RealInput iRefdq[2] annotation(
          Placement(transformation(origin = {-109, 15}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput Eidq[2] annotation(
          Placement(transformation(origin = {171, -17}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput idq0[3] annotation(
          Placement(transformation(origin = {-107, -77}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Continuous.Filter filterd(filterType = Modelica.Blocks.Types.FilterType.LowPass, f_cut = fCutd) annotation(
          Placement(transformation(extent = {{-50, 20}, {-30, 40}})));
        Modelica.Blocks.Continuous.Filter filterq(filterType = Modelica.Blocks.Types.FilterType.LowPass, f_cut = fCutq) annotation(
          Placement(transformation(extent = {{-48, -20}, {-28, 0}})));
        Modelica.Blocks.Routing.Multiplex2 multiplex2 annotation(
          Placement(transformation(extent = {{2, 0}, {22, 20}})));
        Modelica.Blocks.Math.Add add[2] annotation(
          Placement(transformation(extent = {{40, -30}, {60, -10}})));
        Modelica.Blocks.Continuous.LimPID PIDd(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = kpPI, Ti = kpPI/kiPI, yMax = inf, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 0) annotation(
          Placement(transformation(extent = {{90, 34}, {110, 54}})));
        Modelica.Blocks.Continuous.LimPID PIDq(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = kpPI, Ti = kpPI/kiPI, yMax = inf, initType = Modelica.Blocks.Types.Init.InitialOutput, xi_start = 1, y_start = 1, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.Linear) annotation(
          Placement(transformation(extent = {{112, -30}, {132, -10}})));
        Modelica.Blocks.Interfaces.RealInput ivdq[2] annotation(
          Placement(transformation(origin = {-111, -37}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
      equation
        connect(Eidq, Eidq) annotation(
          Line(points = {{171, -17}, {171, -17}}, color = {0, 0, 127}));
        connect(filterd.u, iRefdq[1]) annotation(
          Line(points = {{-52, 30}, {-90, 30}, {-90, 14}, {-100, 14}, {-100, 10.25}, {-109, 10.25}}, color = {0, 0, 127}));
        connect(filterq.u, iRefdq[2]) annotation(
          Line(points = {{-50, -10}, {-90, -10}, {-90, 14}, {-100, 14}, {-100, 19.75}, {-109, 19.75}}, color = {0, 0, 127}));
        connect(filterd.y, multiplex2.u1[1]) annotation(
          Line(points = {{-29, 30}, {-8, 30}, {-8, 16}, {0, 16}}, color = {0, 0, 127}));
        connect(filterq.y, multiplex2.u2[1]) annotation(
          Line(points = {{-27, -10}, {-8, -10}, {-8, 4}, {0, 4}}, color = {0, 0, 127}));
        connect(multiplex2.y, add.u1) annotation(
          Line(points = {{23, 10}, {30, 10}, {30, -14}, {38, -14}}, color = {0, 0, 127}));
        connect(add[1].y, PIDd.u_s) annotation(
          Line(points = {{61, -20}, {78, -20}, {78, 44}, {88, 44}}, color = {0, 0, 127}));
        connect(add[2].y, PIDq.u_s) annotation(
          Line(points = {{61, -20}, {110, -20}}, color = {0, 0, 127}));
        connect(idq0[1], PIDd.u_m) annotation(
          Line(points = {{-107, -83.3333}, {-100, -83.3333}, {-100, -72}, {100, -72}, {100, 32}}, color = {0, 0, 127}));
        connect(idq0[2], PIDq.u_m) annotation(
          Line(points = {{-107, -77}, {122, -77}, {122, -32}}, color = {0, 0, 127}));
        connect(PIDd.y, Eidq[1]) annotation(
          Line(points = {{111, 44}, {150, 44}, {150, -10}, {156, -10}, {156, -19.75}, {171, -19.75}}, color = {0, 0, 127}));
        connect(PIDq.y, Eidq[2]) annotation(
          Line(points = {{133, -20}, {156, -20}, {156, -14.25}, {171, -14.25}}, color = {0, 0, 127}));
        connect(ivdq, add.u2) annotation(
          Line(points = {{-111, -37}, {30, -37}, {30, -26}, {38, -26}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-118, 134}, {114, 108}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-92, 74}, {-18, 48}}, textColor = {0, 0, 0}, textString = "iRefdq"), Text(extent = {{-96, 14}, {-24, -12}}, textColor = {0, 0, 0}, textString = "ivdq"), Text(extent = {{52, 8}, {102, -6}}, textColor = {0, 0, 0}, textString = "Ei*dq"), Text(extent = {{-96, -44}, {-24, -70}}, textColor = {0, 0, 0}, textString = "idq0")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {160, 60}})));
      end CurrentControlVsm;

      model GfmVsmControl
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency fNom = 50 "Frequenza nominale";
        parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Potenza nominale";
        parameter Modelica.Units.SI.Voltage uDc = 750 "Val. nominale tensione DC (per PU)";
        parameter Modelica.Units.SI.Voltage uAcNom = 400 "Val. nominale tensione AC trifase rms (per PU)";
        parameter Modelica.Units.SI.Inductance lf = 5e-4 "Thevenin (circa filter) inductance ";
        final parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom "impedenza base";
        final parameter Modelica.Units.SI.Inductance lNom = zNom/(2*pi*fNom) "induttanza base";
        parameter Modelica.Units.SI.Inductance lfg = 0 "filter LCL grid side inductance";
        parameter Modelica.Units.SI.Inductance lg = 2.44e-5 "grid connection inductance (transformer)";
        parameter Real rv = 0.02 "resistenza virtuale (PU)";
        parameter Real lv = 0.1 "induttanza virtuale (PU)";
        parameter Modelica.Units.SI.Time h = 4 "costante inerzia";
        parameter Real csiMec = 0.7 "smorzamento meccanico swing equation";
        parameter Modelica.Units.SI.Time te = 1 "costante tempo eccitazione";
        parameter Real pGain = 0.05 "P-loop  control gain";
        parameter Real qGain = 0.05 "Q-loop  control gain";
        parameter Modelica.Units.SI.Time fo = 0.02 "first order time constant for P and Q droop";
        Modelica.Blocks.Interfaces.RealInput vabc[3] "Tensioni di fase ai morsetti del Vsc (V)" annotation(
          Placement(transformation(origin = {-105, -85}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput iabc[3] "Correnti di fase ai morsetti del Vsc (V)" annotation(
          Placement(transformation(origin = {-105, -117}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput pRef "Richiesta di potenza attiva (PU)" annotation(
          Placement(transformation(origin = {-102, 88}, extent = {{-18, -18}, {18, 18}}), iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput qRef "Richiesta di potenza reattiva (PU)" annotation(
          Placement(transformation(origin = {-102, 44}, extent = {{-18, -18}, {18, 18}}), iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}})));
        Park.AbcToDq0 toVcdq0 annotation(
          Placement(transformation(origin = {-26, -88}, extent = {{-10, -10}, {10, 10}})));
        Park.AbcToDq0 toIidq0 annotation(
          Placement(transformation(origin = {-24, -120}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.RealExpression thet(y = activeLoopVsm.theta) annotation(
          Placement(transformation(origin = {-76, -146}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput mdqV[2] annotation(
          Placement(transformation(origin = {428, -30}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Gain toVpu[3](k = (1/(uAcNom/sqrt(3)*sqrt(2)))*ones(3)) annotation(
          Placement(transformation(origin = {-64, -84}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Gain toIpu[3](k = (1/(sNom/sqrt(3)/uAcNom*sqrt(2)))*ones(3)) annotation(
          Placement(transformation(origin = {-66, -116}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput pvRef "Richiesta di potenza attiva dal sincrono virtuale (PU)" annotation(
          Placement(transformation(origin = {51, -33}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput qvRef "Richiesta di potenza reattiva dal sincrono virtuale (PU)" annotation(
          Placement(transformation(origin = {113, -107}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}})));
        PowerCalcVsm powerCalcVsm annotation(
          Placement(transformation(extent = {{20, -124}, {80, -72}})));
        ActiveLoopVsm activeLoopVsm(fNom = fNom, h = h) annotation(
          Placement(transformation(extent = {{102, -66}, {150, -22}})));
        ReactiveLoopVsm reactiveLoopVsm(lfg = lfg, lg = lg, lv = lv, te = te) annotation(
          Placement(transformation(extent = {{148, -134}, {194, -96}})));
        CurrentRefCalcVsm currentRefCalcVsm annotation(
          Placement(transformation(extent = {{156, 34}, {222, 76}})));
        StatorEqVsm statorEqVsm(fNom = fNom, lfg = lfg, lg = lg, rv = rv, lv = lv, h = h, csiMec = csiMec) annotation(
          Placement(transformation(extent = {{226, -52}, {276, -10}})));
        Modelica.Blocks.Interfaces.RealOutput theta annotation(
          Placement(transformation(origin = {428, -96}, extent = {{-12, -12}, {12, 12}}), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}})));
        CurrentControlVsm currentControlVsm(fNom = fNom, uAcNom = uAcNom, sNom = sNom, zNom = zNom, lNom = lNom, lf = lf) annotation(
          Placement(transformation(extent = {{310, -52}, {356, -10}})));
        Modelica.Blocks.Math.Gain toV[2](k = (uAcNom/sqrt(3)*sqrt(2))*ones(2)) annotation(
          Placement(transformation(origin = {388, -30}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.RealExpression w(y = activeLoopVsm.w) annotation(
          Placement(transformation(origin = {24, 32}, extent = {{-10, -10}, {10, 10}})));
        DroopVsm droopVsm_RegII(pGain = pGain, qGain = qGain, fo = fo) annotation(
          Placement(transformation(origin = {93, 37}, extent = {{-27, -19}, {27, 19}})));
        Modelica.Blocks.Interfaces.BooleanInput mode "true=island" annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-104, 0}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -100})));
      equation
        connect(toIidq0.wt, thet.y) annotation(
          Line(points = {{-33.8, -124.8}, {-44, -124.8}, {-44, -146}, {-65, -146}}, color = {0, 0, 127}));
        connect(toVcdq0.wt, thet.y) annotation(
          Line(points = {{-35.8, -92.8}, {-44, -92.8}, {-44, -146}, {-65, -146}}, color = {0, 0, 127}));
        connect(toVpu.u, vabc) annotation(
          Line(points = {{-76, -84}, {-78, -85}, {-105, -85}}, color = {0, 0, 127}));
        connect(toVcdq0.abc, toVpu.y) annotation(
          Line(points = {{-36, -84.6}, {-36, -84}, {-53, -84}}, color = {0, 0, 127}));
        connect(toIidq0.abc, toIpu.y) annotation(
          Line(points = {{-34, -116.6}, {-34, -116}, {-55, -116}}, color = {0, 0, 127}));
        connect(toIpu.u, iabc) annotation(
          Line(points = {{-78, -116}, {-82, -117}, {-105, -117}}, color = {0, 0, 127}));
        connect(toVcdq0.dq0, powerCalcVsm.vdq0) annotation(
          Line(points = {{-15.8, -88}, {-6, -88}, {-6, -82.4}, {17, -82.4}}, color = {0, 0, 127}));
        connect(toIidq0.dq0, powerCalcVsm.idq0) annotation(
          Line(points = {{-13.8, -120}, {-2, -120}, {-2, -98}, {17, -98}}, color = {0, 0, 127}));
        connect(pvRef, activeLoopVsm.pvRef) annotation(
          Line(points = {{51, -33}, {52, -33}, {52, -34.32}, {99.12, -34.32}}, color = {0, 0, 127}));
        connect(powerCalcVsm.pv, activeLoopVsm.pv) annotation(
          Line(points = {{81.2, -93.84}, {90, -93.84}, {90, -54.56}, {99.6, -54.56}}, color = {0, 0, 127}));
        connect(reactiveLoopVsm.qvRef, qvRef) annotation(
          Line(points = {{145.24, -106.64}, {113, -106.64}, {113, -107}}, color = {0, 0, 127}));
        connect(powerCalcVsm.qv, reactiveLoopVsm.qv) annotation(
          Line(points = {{81.2, -113.08}, {81.2, -114}, {94, -114}, {94, -124}, {136, -124}, {136, -124.12}, {145.7, -124.12}}, color = {0, 0, 127}));
        connect(statorEqVsm.vdq0, toVcdq0.dq0) annotation(
          Line(points = {{223.5, -18.4}, {206, -18.4}, {206, -8}, {-6, -8}, {-6, -88}, {-15.8, -88}}, color = {0, 0, 127}));
        connect(currentRefCalcVsm.vdq0, toVcdq0.dq0) annotation(
          Line(points = {{152.7, 67.6}, {-6, 67.6}, {-6, -88}, {-15.8, -88}}, color = {0, 0, 127}));
        connect(activeLoopVsm.w, statorEqVsm.w) annotation(
          Line(points = {{150.96, -33.44}, {214, -33.44}, {214, -33.1}, {223.5, -33.1}}, color = {0, 0, 127}));
        connect(reactiveLoopVsm.fe, statorEqVsm.fe) annotation(
          Line(points = {{194.92, -115}, {194.92, -116}, {214, -116}, {214, -47.38}, {224, -47.38}}, color = {0, 0, 127}));
        connect(currentRefCalcVsm.iRefdq, currentControlVsm.iRefdq) annotation(
          Line(points = {{223.32, 55}, {296, 55}, {296, -18.4}, {305.4, -18.4}}, color = {0, 0, 127}));
        connect(statorEqVsm.ivdq, currentControlVsm.ivdq) annotation(
          Line(points = {{277.5, -31}, {292, -31}, {292, -31}, {305.4, -31}}, color = {0, 0, 127}));
        connect(currentControlVsm.idq0, toIidq0.dq0) annotation(
          Line(points = {{305.4, -43.6}, {298, -43.6}, {298, -152}, {-4, -152}, {-4, -120}, {-13.8, -120}}, color = {0, 0, 127}));
        connect(activeLoopVsm.theta, theta) annotation(
          Line(points = {{150.96, -54.12}, {176, -54.12}, {176, -54}, {206, -54}, {206, -96}, {428, -96}}, color = {0, 0, 127}));
        connect(currentControlVsm.Eidq, toV.u) annotation(
          Line(points = {{356.92, -31}, {356.92, -30}, {376, -30}}, color = {0, 0, 127}));
        connect(toV.y, mdqV) annotation(
          Line(points = {{399, -30}, {428, -30}}, color = {0, 0, 127}));
        connect(statorEqVsm.ivdq, powerCalcVsm.ivdq) annotation(
          Line(points = {{277.5, -31}, {286, -31}, {286, -142}, {4, -142}, {4, -113.6}, {17, -113.6}}, color = {0, 0, 127}));
        connect(w.y, droopVsm_RegII.w) annotation(
          Line(points = {{35, 32}, {48, 32}, {48, 31.3}, {60.6, 31.3}}, color = {0, 0, 127}));
        connect(droopVsm_RegII.qRef, qRef) annotation(
          Line(points = {{60.6, 42.7}, {-102, 42.7}, {-102, 44}}, color = {0, 0, 127}));
        connect(droopVsm_RegII.pRef, pRef) annotation(
          Line(points = {{60.6, 52.2}, {-58, 52.2}, {-58, 88}, {-102, 88}}, color = {0, 0, 127}));
        connect(droopVsm_RegII.pRefDroop, currentRefCalcVsm.pRef) annotation(
          Line(points = {{121.08, 44.22}, {134, 44.22}, {134, 55}, {152.7, 55}}, color = {0, 0, 127}));
        connect(droopVsm_RegII.qRefDroop, currentRefCalcVsm.qRef) annotation(
          Line(points = {{121.08, 30.92}, {152.7, 30.92}, {152.7, 42.4}}, color = {0, 0, 127}));
        connect(toVcdq0.dq0, droopVsm_RegII.vdq0) annotation(
          Line(points = {{-15.8, -88}, {-6, -88}, {-6, 14}, {50, 14}, {50, 21.8}, {60.6, 21.8}}, color = {0, 0, 127}));
        connect(mode, droopVsm_RegII.mode) annotation(
          Line(points = {{-104, 0}, {98.4, 0}, {98.4, 18}}, color = {255, 0, 255}));
        annotation(
          Diagram(coordinateSystem(extent = {{-100, -160}, {420, 100}})),
          Icon(graphics = {Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 124}, {100, 100}}, textString = "%name"), Text(textColor = {238, 46, 47}, extent = {{-64, 22}, {72, -28}}, textString = "VSM
      control"), Text(extent = {{56, -46}, {96, -60}}, textString = "q", fontName = "Symbol", textColor = {0, 0, 0}, textStyle = {TextStyle.Italic}), Text(extent = {{-106, 96}, {-56, 82}}, textString = "v", textColor = {0, 0, 0}), Text(extent = {{-106, 62}, {-56, 48}}, textString = "i", textColor = {0, 0, 0}), Text(extent = {{-100, 26}, {-50, 12}}, textString = "pRef", textColor = {0, 0, 0}), Text(extent = {{-102, -14}, {-52, -28}}, textColor = {0, 0, 0}, textString = "qRef"), Text(extent = {{46, 68}, {100, 54}}, textColor = {0, 0, 0}, textString = "mdqV"), Text(extent = {{-102, -48}, {-52, -62}}, textColor = {0, 0, 0}, textString = "pvRef"), Text(extent = {{-102, -82}, {-52, -96}}, textColor = {0, 0, 0}, textString = "qvRef")}));
      end GfmVsmControl;

      block DroopVsm
        import Modelica.Constants.pi;
        import Modelica.Constants.inf;
        parameter Real pGain = 0.05 "P-loop  control gain";
        parameter Real qGain = 0.05 "Q-loop  control gain";
        parameter Modelica.Units.SI.Time fo = 0.02 "first order time constant for P and Q droop";
        Modelica.Blocks.Interfaces.RealInput vdq0[3] annotation(
          Placement(transformation(origin = {-167, -113}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput w annotation(
          Placement(transformation(origin = {-110, 40}, extent = {{-18, -18}, {18, 18}}), iconTransformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Math.Add add1(k2 = -1) annotation(
          Placement(transformation(extent = {{-14, 30}, {6, 50}})));
        Modelica.Blocks.Interfaces.RealOutput pRefDroop annotation(
          Placement(transformation(origin = {149, 47}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, 38}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput qRefDroop annotation(
          Placement(transformation(origin = {147, -103}, extent = {{-11, -11}, {11, 11}}), iconTransformation(origin = {104, -32}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Constant const(k = 1) annotation(
          Placement(transformation(extent = {{-68, 72}, {-48, 92}})));
        Modelica.Blocks.Math.Gain gainP(k = 1/pGain) annotation(
          Placement(transformation(extent = {{22, 30}, {42, 50}})));
        Modelica.Blocks.Math.Gain gainQ(k = 1/qGain) annotation(
          Placement(transformation(origin = {16, -52}, extent = {{22, -56}, {42, -36}})));
        Modelica.Blocks.Sources.Constant const1(k = 1) annotation(
          Placement(transformation(extent = {{-108, -86}, {-88, -66}})));
        Modelica.Blocks.Math.Add add2(k2 = -1) annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-16, -56}, {4, -36}})));
        Modelica.Blocks.Interfaces.RealInput pRef annotation(
          Placement(transformation(origin = {11, 79}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput qRef annotation(
          Placement(transformation(origin = {65, -133}, extent = {{-19, -19}, {19, 19}}), iconTransformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Math.Add add3 annotation(
          Placement(transformation(origin = {48, 0}, extent = {{60, 36}, {80, 56}})));
        Modelica.Blocks.Math.Add add4 annotation(
          Placement(transformation(origin = {36, -52}, extent = {{66, -62}, {86, -42}})));
        Modelica.Blocks.Continuous.FirstOrder PfirstOrder(T = fo) annotation(
          Placement(transformation(extent = {{-64, 30}, {-44, 50}})));
        Modelica.Blocks.Continuous.FirstOrder QfirstOrder(T = fo, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 1) annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-44, -72}, {-24, -52}})));
        Modelica.Blocks.Math.Sqrt sqrt1 annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-72, -72}, {-52, -52}})));
        Modelica.Blocks.Math.Product product1 annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-132, -60}, {-112, -40}})));
        Modelica.Blocks.Math.Product product2 annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-132, -86}, {-112, -66}})));
        Modelica.Blocks.Math.Add add annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-100, -72}, {-80, -52}})));
        Modelica.Blocks.Math.Add add5 annotation(
          Placement(transformation(origin = {10, -20}, extent = {{60, 36}, {80, 56}})));
        Modelica.Blocks.Continuous.LimIntegrator limIntegrator(k = 1/pGain, outMax = inf, use_reset = false, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 0) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {50, 8})));
        Modelica.Blocks.Sources.BooleanStep booleanStep(each startTime = 2.1) annotation(
          Placement(transformation(extent = {{-72, -54}, {-52, -34}})));
        Modelica.Blocks.Logical.Switch switch1 annotation(
          Placement(transformation(extent = {{12, -2}, {32, 18}})));
        Modelica.Blocks.Sources.Constant const2(k = 0) annotation(
          Placement(transformation(extent = {{-20, -20}, {0, 0}})));
        Modelica.Blocks.Continuous.LimIntegrator limIntegrator1(k = 1/qGain, outMax = inf, use_reset = false, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 0) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {52, -52})));
        Modelica.Blocks.Math.Add add6 annotation(
          Placement(transformation(origin = {18, -114}, extent = {{60, 36}, {80, 56}})));
        Modelica.Blocks.Logical.Switch switch2 annotation(
          Placement(transformation(extent = {{10, -42}, {30, -62}})));
        Modelica.Blocks.Interfaces.BooleanInput mode "true=island" annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 0, origin = {-112, -22}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {20, -100})));
      equation
        connect(pRefDroop, pRefDroop) annotation(
          Line(points = {{149, 47}, {149, 47}}, color = {0, 0, 127}));
        connect(const.y, add1.u1) annotation(
          Line(points = {{-47, 82}, {-24, 82}, {-24, 46}, {-16, 46}}, color = {0, 0, 127}));
        connect(add1.y, gainP.u) annotation(
          Line(points = {{7, 40}, {20, 40}}, color = {0, 0, 127}));
        connect(const1.y, add2.u1) annotation(
          Line(points = {{-87, -76}, {-26, -76}, {-26, -92}, {-18, -92}}, color = {0, 0, 127}));
        connect(add2.y, gainQ.u) annotation(
          Line(points = {{5, -98}, {36, -98}}, color = {0, 0, 127}));
        connect(pRef, add3.u1) annotation(
          Line(points = {{11, 79}, {52, 79}, {52, 52}, {106, 52}}, color = {0, 0, 127}));
        connect(qRef, add4.u2) annotation(
          Line(points = {{65, -133}, {66, -133}, {66, -110}, {100, -110}}, color = {0, 0, 127}));
        connect(add4.y, qRefDroop) annotation(
          Line(points = {{123, -104}, {126, -103}, {147, -103}}, color = {0, 0, 127}));
        connect(add3.y, pRefDroop) annotation(
          Line(points = {{129, 46}, {138, 46}, {138, 47}, {149, 47}}, color = {0, 0, 127}));
        connect(w, PfirstOrder.u) annotation(
          Line(points = {{-110, 40}, {-66, 40}}, color = {0, 0, 127}));
        connect(PfirstOrder.y, add1.u2) annotation(
          Line(points = {{-43, 40}, {-26, 40}, {-26, 34}, {-16, 34}}, color = {0, 0, 127}));
        connect(vdq0[1], product1.u1) annotation(
          Line(points = {{-167, -119.333}, {-144, -119.333}, {-144, -96}, {-134, -96}}, color = {0, 0, 127}));
        connect(vdq0[1], product1.u2) annotation(
          Line(points = {{-167, -119.333}, {-144, -119.333}, {-144, -108}, {-134, -108}}, color = {0, 0, 127}));
        connect(QfirstOrder.u, sqrt1.y) annotation(
          Line(points = {{-46, -114}, {-51, -114}}, color = {0, 0, 127}));
        connect(vdq0[2], product2.u2) annotation(
          Line(points = {{-167, -113}, {-144, -113}, {-144, -134}, {-134, -134}}, color = {0, 0, 127}));
        connect(product1.y, add.u1) annotation(
          Line(points = {{-111, -102}, {-110, -102}, {-110, -108}, {-102, -108}}, color = {0, 0, 127}));
        connect(product2.y, add.u2) annotation(
          Line(points = {{-111, -128}, {-110, -128}, {-110, -124}, {-102, -124}, {-102, -120}}, color = {0, 0, 127}));
        connect(sqrt1.u, add.y) annotation(
          Line(points = {{-74, -114}, {-79, -114}}, color = {0, 0, 127}));
        connect(QfirstOrder.y, add2.u2) annotation(
          Line(points = {{-23, -114}, {-22, -114}, {-22, -104}, {-18, -104}}, color = {0, 0, 127}));
        connect(qRefDroop, qRefDroop) annotation(
          Line(points = {{147, -103}, {147, -103}}, color = {0, 0, 127}));
        connect(add5.y, add3.u2) annotation(
          Line(points = {{91, 26}, {106, 26}, {106, 40}}, color = {0, 0, 127}));
        connect(gainP.y, add5.u1) annotation(
          Line(points = {{43, 40}, {68, 40}, {68, 32}}, color = {0, 0, 127}));
        connect(switch1.y, limIntegrator.u) annotation(
          Line(points = {{33, 8}, {38, 8}}, color = {0, 0, 127}));
        connect(switch1.u1, add1.y) annotation(
          Line(points = {{10, 16}, {10, 36}, {12, 36}, {12, 40}, {7, 40}}, color = {0, 0, 127}));
        connect(switch1.u3, const2.y) annotation(
          Line(points = {{10, 0}, {2, 0}, {2, -10}, {1, -10}}, color = {0, 0, 127}));
        connect(limIntegrator.y, add5.u2) annotation(
          Line(points = {{61, 8}, {68, 8}, {68, 20}}, color = {0, 0, 127}));
        connect(product2.u1, vdq0[2]) annotation(
          Line(points = {{-134, -122}, {-144, -122}, {-144, -113}, {-167, -113}}, color = {0, 0, 127}));
        connect(limIntegrator1.y, add6.u1) annotation(
          Line(points = {{63, -52}, {66, -52}, {66, -62}, {76, -62}}, color = {0, 0, 127}));
        connect(limIntegrator1.u, switch2.y) annotation(
          Line(points = {{40, -52}, {31, -52}}, color = {0, 0, 127}));
        connect(add2.y, switch2.u1) annotation(
          Line(points = {{5, -98}, {10, -98}, {10, -68}, {8, -68}, {8, -60}}, color = {0, 0, 127}));
        connect(switch2.u3, const2.y) annotation(
          Line(points = {{8, -44}, {8, -10}, {1, -10}}, color = {0, 0, 127}));
        connect(gainQ.y, add6.u2) annotation(
          Line(points = {{59, -98}, {68, -98}, {68, -74}, {76, -74}}, color = {0, 0, 127}));
        connect(add6.y, add4.u1) annotation(
          Line(points = {{99, -68}, {104, -68}, {104, -90}, {100, -90}, {100, -98}}, color = {0, 0, 127}));
        connect(mode, switch1.u2) annotation(
          Line(points = {{-112, -22}, {-112, 8}, {10, 8}}, color = {255, 0, 255}));
        connect(mode, switch2.u2) annotation(
          Line(points = {{-112, -22}, {-24, -22}, {-24, -52}, {8, -52}}, color = {255, 0, 255}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {Text(textColor = {0, 0, 255}, extent = {{-118, 128}, {114, 102}}, textString = "%name"), Rectangle(lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-106, -14}, {-30, -42}}, textString = "w"), Text(extent = {{-94, -68}, {-26, -94}}, textString = "vdq0"), Text(extent = {{-2, 54}, {88, 24}}, textString = "pRefDroop"), Text(extent = {{0, -16}, {90, -46}}, textString = "qRefDroop"), Text(extent = {{-98, 90}, {-22, 62}}, textString = "pRef"), Text(extent = {{-100, 46}, {-24, 18}}, textString = "qRef")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -140}, {140, 100}}), graphics = {Text(textColor = {238, 46, 47}, extent = {{22, -10}, {92, -24}}, textStyle = {TextStyle.Bold}, textString = "costante tempo secondaria
come in gfm unipi
da rivedere")}));
      end DroopVsm;
    end Vsm;

    package Park
      model AbcToDq
        Modelica.Blocks.Interfaces.RealInput abc[3] annotation(
          Placement(transformation(origin = {-108, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 34}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput wt annotation(
          Placement(transformation(origin = {-108, -38}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -48}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor annotation(
          Placement(transformation(origin = {-38, 50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator annotation(
          Placement(transformation(origin = {26, 62}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput dq[2] annotation(
          Placement(transformation(origin = {106, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(rotator.angle, wt) annotation(
          Line(points = {{26, 50}, {26, -38}, {-108, -38}}, color = {0, 0, 127}));
        connect(rotator.y[1:2], dq[1:2]) annotation(
          Line(points = {{37, 62}, {72, 62}, {72, 24.5}, {106, 24.5}}, color = {0, 0, 127}));
        connect(rotator.u, toSpacePhasor.y) annotation(
          Line(points = {{14, 62}, {-6, 62}, {-6, 50}, {-27, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(toSpacePhasor.u, abc) annotation(
          Line(points = {{-50, 50}, {-108, 50}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-116, 148}, {104, 108}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-100, 50}, {98, -32}}, textColor = {238, 46, 47}, textString = "To
dq"), Text(extent = {{-72, -44}, {-58, -56}}, textColor = {0, 0, 0}, horizontalAlignment = TextAlignment.Left, fontName = "Symbol", textString = "q", textStyle = {TextStyle.Italic})}));
      end AbcToDq;

      model DqToAbc
        Modelica.Blocks.Interfaces.RealInput dq[2] annotation(
          Placement(transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, 32}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.FromSpacePhasor fromSpacePhasor annotation(
          Placement(transformation(origin = {30, 8}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator annotation(
          Placement(transformation(origin = {-44, 38}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput theta annotation(
          Placement(transformation(origin = {-108, -48}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, -56}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput abc[3] annotation(
          Placement(transformation(origin = {106, 8}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(transformation(extent = {{-56, -80}, {-36, -60}})));
        Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {4, -28})));
      equation
        connect(rotator.y, fromSpacePhasor.u) annotation(
          Line(points = {{-33, 38}, {-4, 38}, {-4, 8}, {18, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(fromSpacePhasor.y, abc) annotation(
          Line(points = {{41, 8}, {106, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(dq[1:2], rotator.u) annotation(
          Line(points = {{-108, 5}, {-70, 5}, {-70, 38}, {-56, 38}}, color = {0, 0, 127}, thickness = 0.5));
        connect(gain.u, theta) annotation(
          Line(points = {{-58, -70}, {-82, -70}, {-82, -48}, {-108, -48}}, color = {0, 0, 127}));
        connect(gain.y, rotator.angle) annotation(
          Line(points = {{-35, -70}, {-32, -70}, {-32, 16}, {-44, 16}, {-44, 26}}, color = {0, 0, 127}));
        connect(const1.y, fromSpacePhasor.zero) annotation(
          Line(points = {{4, -17}, {4, 0}, {18, 0}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-100, 144}, {98, 104}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-100, 44}, {98, -38}}, textColor = {238, 46, 47}, textString = "To
abc"), Text(extent = {{-72, -52}, {-32, -66}}, textColor = {0, 0, 0}, textString = "q", fontName = "Symbol", textStyle = {TextStyle.Italic}), Text(extent = {{-90, 40}, {-24, 26}}, textColor = {0, 0, 0}, textString = "dq")}));
      end DqToAbc;

      model DToAbc
        Modelica.Blocks.Interfaces.RealInput d annotation(
          Placement(transformation(origin = {-112, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, 60}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.FromSpacePhasor fromSpacePhasor annotation(
          Placement(transformation(origin = {50, 8}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator annotation(
          Placement(transformation(origin = {-30, 38}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput theta annotation(
          Placement(transformation(origin = {-108, -48}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, -56}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput abc[3] annotation(
          Placement(transformation(origin = {106, 8}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(transformation(extent = {{-60, -58}, {-40, -38}})));
        Modelica.Blocks.Sources.Constant const(k = 0) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-64, 2})));
        Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {24, -18})));
      equation
        connect(rotator.y, fromSpacePhasor.u) annotation(
          Line(points = {{-19, 38}, {8, 38}, {8, 8}, {38, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(fromSpacePhasor.y, abc) annotation(
          Line(points = {{61, 8}, {106, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(gain.u, theta) annotation(
          Line(points = {{-62, -48}, {-108, -48}}, color = {0, 0, 127}));
        connect(gain.y, rotator.angle) annotation(
          Line(points = {{-39, -48}, {-30, -48}, {-30, 26}}, color = {0, 0, 127}));
        connect(rotator.u[1], d) annotation(
          Line(points = {{-42, 38}, {-74, 38}, {-74, 60}, {-112, 60}}, color = {0, 0, 127}));
        connect(const.y, rotator.u[2]) annotation(
          Line(points = {{-64, 13}, {-64, 38}, {-42, 38}}, color = {0, 0, 127}));
        connect(fromSpacePhasor.zero, const1.y) annotation(
          Line(points = {{38, 0}, {24, 0}, {24, -7}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-100, 144}, {98, 104}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-90, 48}, {108, -34}}, textColor = {238, 46, 47}, textString = "To
abc"), Text(extent = {{-74, -50}, {-42, -66}}, textColor = {0, 0, 0}, fontName = "Symbol", textString = "q", textStyle = {TextStyle.Italic}), Text(extent = {{-92, 66}, {-26, 52}}, textColor = {0, 0, 0}, textString = "d")}));
      end DToAbc;

      model Dq0ToAbc
        Modelica.Blocks.Interfaces.RealInput dq0[3] annotation(
          Placement(transformation(origin = {-108, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, 32}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.FromSpacePhasor fromSpacePhasor annotation(
          Placement(transformation(origin = {30, 8}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator annotation(
          Placement(transformation(origin = {-44, 38}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealInput theta annotation(
          Placement(transformation(origin = {-108, -48}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-92, -56}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealOutput abc[3] annotation(
          Placement(transformation(origin = {106, 8}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Math.Gain gain(k = -1) annotation(
          Placement(transformation(extent = {{-56, -80}, {-36, -60}})));
      equation
        connect(fromSpacePhasor.zero, dq0[3]) annotation(
          Line(points = {{18, 0}, {-46, 0}, {-46, 6.66667}, {-108, 6.66667}}, color = {0, 0, 127}));
        connect(rotator.y, fromSpacePhasor.u) annotation(
          Line(points = {{-33, 38}, {-4, 38}, {-4, 8}, {18, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(fromSpacePhasor.y, abc) annotation(
          Line(points = {{41, 8}, {106, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(dq0[1:2], rotator.u) annotation(
          Line(points = {{-108, 0}, {-70, 0}, {-70, 38}, {-56, 38}}, color = {0, 0, 127}, thickness = 0.5));
        connect(gain.u, theta) annotation(
          Line(points = {{-58, -70}, {-82, -70}, {-82, -48}, {-108, -48}}, color = {0, 0, 127}));
        connect(gain.y, rotator.angle) annotation(
          Line(points = {{-35, -70}, {-32, -70}, {-32, 16}, {-44, 16}, {-44, 26}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-100, 144}, {98, 104}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-100, 44}, {98, -38}}, textColor = {238, 46, 47}, textString = "To
abc"), Text(extent = {{-72, -52}, {-32, -66}}, textColor = {0, 0, 0}, textString = "q", fontName = "Symbol", textStyle = {TextStyle.Italic}), Text(extent = {{-92, 40}, {-26, 26}}, textColor = {0, 0, 0}, textString = "dq0")}));
      end Dq0ToAbc;

      model AbcToDq0
        Modelica.Blocks.Interfaces.RealInput abc[3] annotation(
          Placement(transformation(origin = {-108, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 34}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Blocks.Interfaces.RealInput wt annotation(
          Placement(transformation(origin = {-108, -38}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -48}, extent = {{-20, -20}, {20, 20}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor annotation(
          Placement(transformation(origin = {-38, 50}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator annotation(
          Placement(transformation(origin = {26, 62}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.RealOutput dq0[3] annotation(
          Placement(transformation(origin = {106, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(rotator.angle, wt) annotation(
          Line(points = {{26, 50}, {26, -38}, {-108, -38}}, color = {0, 0, 127}));
        connect(rotator.y[1:2], dq0[1:2]) annotation(
          Line(points = {{37, 62}, {72, 62}, {72, 22}, {106, 22}}, color = {0, 0, 127}));
        connect(rotator.u, toSpacePhasor.y) annotation(
          Line(points = {{14, 62}, {-6, 62}, {-6, 50}, {-27, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(toSpacePhasor.u, abc) annotation(
          Line(points = {{-50, 50}, {-108, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(toSpacePhasor.zero, dq0[3]) annotation(
          Line(points = {{-27, 42}, {-22, 42}, {-22, 8}, {106, 8}, {106, 25.3333}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-116, 148}, {104, 108}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-94, 62}, {104, -20}}, textColor = {238, 46, 47}, textString = "To
dq0"), Text(extent = {{-72, -44}, {-58, -56}}, textColor = {0, 0, 0}, horizontalAlignment = TextAlignment.Left, fontName = "Symbol", textString = "q", textStyle = {TextStyle.Italic})}));
      end AbcToDq0;
    end Park;

    package PLLdedicated
      block VarAvg "Sensor to measure the average value of input - variable frequency"
        Modelica.Blocks.Interfaces.RealInput u annotation(
          Placement(transformation(extent = {{-140, 40}, {-100, 80}})));
        Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(transformation(extent = {{100, -10}, {120, 10}})));
        Modelica.Blocks.Continuous.Integrator integrator annotation(
          Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
        Modelica.Blocks.Math.Add add(k2 = -1) annotation(
          Placement(transformation(extent = {{12, -10}, {32, 10}})));
        parameter Modelica.Units.SI.Frequency fMin = 49 "Frequency of the signals to be averaged";
        Modelica.Blocks.Nonlinear.VariableDelay variableDelay(delayMax = 1.02/fMin) annotation(
          Placement(transformation(extent = {{-26, -36}, {-6, -16}})));
        Modelica.Blocks.Interfaces.RealInput f annotation(
          Placement(transformation(extent = {{-138, -80}, {-98, -40}})));
        Modelica.Blocks.Math.Product product annotation(
          Placement(transformation(extent = {{54, -10}, {74, 10}})));
        Modelica.Blocks.Sources.RealExpression realExpression(y = 1/f) annotation(
          Placement(transformation(extent = {{-66, -44}, {-46, -24}})));
      equation
        connect(integrator.u, u) annotation(
          Line(points = {{-62, 0}, {-92, 0}, {-92, 60}, {-120, 60}}, color = {0, 0, 127}, smooth = Smooth.None));
        connect(add.u1, integrator.y) annotation(
          Line(points = {{10, 6}, {-14, 6}, {-14, 0}, {-39, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
        connect(variableDelay.y, add.u2) annotation(
          Line(points = {{-5, -26}, {2, -26}, {2, -6}, {10, -6}}, color = {0, 0, 127}));
        connect(variableDelay.u, integrator.y) annotation(
          Line(points = {{-28, -26}, {-34, -26}, {-34, 0}, {-39, 0}}, color = {0, 0, 127}));
        connect(add.y, product.u1) annotation(
          Line(points = {{33, 0}, {38, 0}, {38, 6}, {52, 6}}, color = {0, 0, 127}));
        connect(product.u2, f) annotation(
          Line(points = {{52, -6}, {40, -6}, {40, -60}, {-118, -60}}, color = {0, 0, 127}));
        connect(product.y, y) annotation(
          Line(points = {{75, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(realExpression.y, variableDelay.delayTime) annotation(
          Line(points = {{-45, -34}, {-38, -34}, {-38, -32}, {-28, -32}}, color = {0, 0, 127}));
        annotation(
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, grid = {2, 2})),
          Documentation(info = "<html><p>
 This power sensor measures instantaneous electrical power of a singlephase system and has a separated voltage and current path. The pins of the voltage path are <code>pv</code> and <code>nv</code>, the pins of the current path are <code>pc</code> and <code>nc</code>. The internal resistance of the current path is zero, the internal resistance of the voltage path is infinite.
 </p>
 </html>", revisions = "<html>
 <ul>
 <li><i> January 12, 2006   </i>
        by Anton Haumer<br> implemented<br>
        </li>
 </ul>
 </html>"),
          Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {2, 2}), graphics = {Text(lineColor = {0, 0, 255}, extent = {{100, 112}, {-104, 146}}, textString = "%name"), Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 102}, {100, -100}}), Text(lineColor = {0, 0, 127}, extent = {{-100, 44}, {100, -44}}, textString = "var
AVG")}));
      end VarAvg;

      model Mod
        extends Modelica.Blocks.Interfaces.SISO;
        parameter Real A = 2*Modelica.Constants.pi;
      equation
        y = mod(u, A);
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(extent = {{-100, 10}, {100, -20}}, lineColor = {0, 0, 255}, textString = "mod(u/A)")}),
          Diagram(coordinateSystem(preserveAspectRatio = false)));
      end Mod;

      model PLL
        import Modelica.Constants.pi;
        parameter Real Kp(unit = "rad/(s.V)") = 30 "PI proportional gain";
        parameter Modelica.Units.SI.Voltage vNom = 1 "expected voltage amplitude";
        parameter Modelica.Units.SI.Time Ti = 60/1400 "PI time constant";
        parameter Modelica.Units.SI.Frequency fMin = 49 "minimum frequency to be measured";
        parameter Modelica.Units.SI.Frequency fInit = 50 "initial frequency";
        parameter Modelica.Units.SI.Frequency fMax = 51 "maximum frequency to be measured";
        parameter Real maxSlew = 1 "max frequency variation (Hz/s)";
        Modelica.Blocks.Continuous.PI PIcontr(T = 60/1400, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 2.0*pi*fInit, k = 60/vNom) annotation(
          Placement(transformation(extent = {{-52, -10}, {-32, 10}})));
        Modelica.Blocks.Math.Gain toRad(k = 1.0/(2.0*pi)) annotation(
          Placement(transformation(extent = {{-14, 30}, {6, 50}})));
        Modelica.Blocks.Continuous.Filter filter(order = 2, f_cut = 30, analogFilter = Modelica.Blocks.Types.AnalogFilter.CriticalDamping) annotation(
          Placement(transformation(extent = {{52, 30}, {72, 50}})));
        Modelica.Blocks.Interfaces.RealOutput fHz annotation(
          Placement(transformation(extent = {{100, 50}, {120, 70}}), iconTransformation(extent = {{100, 50}, {120, 70}})));
        Modelica.Blocks.Interfaces.RealOutput thetaRad annotation(
          Placement(transformation(extent = {{100, -70}, {120, -50}}), iconTransformation(extent = {{100, -70}, {120, -50}})));
        Modelica.Blocks.Continuous.Integrator integrator(y_start = 3/2*pi) annotation(
          Placement(transformation(extent = {{-4, -50}, {16, -30}})));
        PLLdedicated.Mod mod annotation(
          Placement(transformation(extent = {{32, -50}, {52, -30}})));
        Modelica.Blocks.Interfaces.RealInput u[3] annotation(
          Placement(visible = true, transformation(extent = {{-180, -20}, {-140, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
        PLLdedicated.VarAvg varAvg(fMin = fMin) annotation(
          Placement(transformation(extent = {{-84, 10}, {-64, -10}})));
        Modelica.Blocks.Nonlinear.Limiter limiter(uMax = fMax, uMin = fMin) annotation(
          Placement(transformation(extent = {{-56, 34}, {-76, 54}})));
        Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = maxSlew) annotation(
          Placement(transformation(extent = {{18, 30}, {38, 50}})));
        Park.AbcToDq abcToDq annotation(
          Placement(transformation(extent = {{-118, -16}, {-98, 4}})));
      equation
        connect(toRad.u, PIcontr.y) annotation(
          Line(points = {{-16, 40}, {-22, 40}, {-22, 0}, {-31, 0}}, color = {0, 0, 127}));
        connect(filter.y, fHz) annotation(
          Line(points = {{73, 40}, {92, 40}, {92, 60}, {110, 60}}, color = {0, 0, 127}));
        connect(integrator.u, PIcontr.y) annotation(
          Line(points = {{-6, -40}, {-22, -40}, {-22, 0}, {-31, 0}}, color = {0, 0, 127}));
        connect(mod.u, integrator.y) annotation(
          Line(points = {{30, -40}, {17, -40}}, color = {0, 0, 127}));
        connect(mod.y, thetaRad) annotation(
          Line(points = {{53, -40}, {82, -40}, {82, -60}, {110, -60}}, color = {0, 0, 127}));
        connect(limiter.u, fHz) annotation(
          Line(points = {{-54, 44}, {-40, 44}, {-40, 60}, {110, 60}}, color = {0, 0, 127}));
        connect(limiter.y, varAvg.f) annotation(
          Line(points = {{-77, 44}, {-94, 44}, {-94, 6}, {-85.8, 6}}, color = {0, 0, 127}));
        connect(slewRateLimiter.y, filter.u) annotation(
          Line(points = {{39, 40}, {50, 40}}, color = {0, 0, 127}));
        connect(slewRateLimiter.u, toRad.y) annotation(
          Line(points = {{16, 40}, {7, 40}}, color = {0, 0, 127}));
        connect(varAvg.y, PIcontr.u) annotation(
          Line(points = {{-63, 0}, {-54, 0}}, color = {0, 0, 127}));
        connect(abcToDq.dq[2], varAvg.u) annotation(
          Line(points = {{-97.8, -5.75}, {-97.8, -6}, {-86, -6}}, color = {0, 0, 127}));
        connect(abcToDq.abc, u) annotation(
          Line(points = {{-118, -2.6}, {-136, -2.6}, {-136, 0}, {-160, 0}}, color = {0, 0, 127}));
        connect(abcToDq.wt, thetaRad) annotation(
          Line(points = {{-117.8, -10.8}, {-126, -10.8}, {-126, -60}, {110, -60}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics = {Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 127}, extent = {{-100, 20}, {100, -20}}, textString = "PLL"), Text(origin = {1.0297, -10.4}, textColor = {0, 0, 255}, extent = {{-105.03, 156.4}, {98.9703, 122.4}}, textString = "%name"), Text(origin = {81, 60}, textColor = {0, 0, 127}, extent = {{-17, 14}, {17, -14}}, textString = "f", fontName = "Times", textStyle = {TextStyle.Italic}), Text(origin = {78, -58}, textColor = {0, 0, 127}, extent = {{-18, 14}, {18, -14}}, textString = "q", fontName = "Symbol", textStyle = {TextStyle.Italic})}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -80}, {100, 80}}, initialScale = 0.1)));
      end PLL;
    end PLLdedicated;
  end Blocks;

  package Electric
    model Inv
      parameter Modelica.Units.SI.Time delay(start = 0.001) "internal first-order delay time";
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug annotation(
        Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(
        Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(
        Placement(transformation(origin = {-100, -62}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, -62}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealInput u[3] annotation(
        Placement(transformation(origin = {0, -114}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -112}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
      Modelica.Units.SI.Power acPow "AC power, positive when entering the model", dcPow "DC power, positive when entering the model";
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_ac annotation(
        Placement(transformation(origin = {100, -62}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {98, -60}, extent = {{-10, -10}, {10, 10}})));
    equation
//Kirchhoff balance:
      pin_p.i + pin_n.i = 0;
      plug.pin[1].i + plug.pin[2].i + plug.pin[3].i + pin_ac.i = 0;
//Apply input voltages:
      for i in 1:3 loop
        plug.pin[i].v - pin_ac.v = u[i];
      end for;
//Power balance (three equations; entering DC power follows exiting AC's) :
      acPow = plug.pin[1].i*(plug.pin[1].v - pin_ac.v) + plug.pin[2].i*(plug.pin[2].v - pin_ac.v) + plug.pin[3].i*(plug.pin[3].v - pin_ac.v);
      dcPow = pin_p.i*(pin_p.v - pin_n.v);
      delay*der(dcPow) + dcPow = -acPow;
      annotation(
        Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-106, 146}, {104, 104}}, textString = "%name"), Line(points = {{-98, -98}, {100, 98}}, color = {28, 108, 200}), Line(points = {{-62, 52}, {-16, 52}}), Line(points = {{10, -56}, {24, -36}, {34, -36}, {50, -74}, {59.541, -74}, {72, -54}}), Text(origin = {0, 1}, textColor = {238, 46, 47}, extent = {{-100, 19}, {98, -17}}, textString = "LossLess")}));
    end Inv;

    model DistortedGrid
      import Modelica.Constants.pi;
      parameter Modelica.Units.SI.Frequency f1 = 50 "nominal frequency";
      parameter Modelica.Units.SI.Frequency f2 = 49.5 "second frequency";
      parameter Modelica.Units.SI.Time startF2Time = 1000 "start time of frequency ramp f1->f2";
      parameter Modelica.Units.SI.Time rampF2duration = 1 "ramp f1->f2 duration";
      parameter Modelica.Units.SI.Voltage ampl = 400*sqrt(2/3) "nominal voltage amplitude";
      parameter Modelica.Units.SI.Voltage ampl2 = 1.1*400*sqrt(2/3) "second voltage amplitude";
      parameter Modelica.Units.SI.Time startV2Time = 1000 "start time of voltage amplitude ramp ampl->ampl2";
      parameter Modelica.Units.SI.Time rampV2duration = 1 "ramp ampl->ampl2 duration";
      parameter Modelica.Units.SI.Voltage ampl15 = 0.05*ampl "15th harmonic amplitude";
      parameter Modelica.Units.SI.Voltage ampl17 = 0.03*ampl "17th harmonic amplitude";
      parameter Modelica.Units.SI.Time startHTime = 1000 "start of harmonics injection";
      parameter Modelica.Units.SI.Resistance R "transformer equivalent resistance";
      parameter Modelica.Units.SI.Inductance L "transformer equivalent inductance";
      Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage annotation(
        Placement(transformation(origin = {-14, 38}, extent = {{34, -48}, {54, -28}})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {62, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Blocks.Sources.Ramp ramp(height = f2 - f1, duration = rampF2duration, offset = f1, startTime = startF2Time) annotation(
        Placement(transformation(extent = {{-90, 82}, {-70, 102}})));
      Modelica.Blocks.Sources.SineVariableFrequencyAndAmplitude sine1(phi(fixed = true, displayUnit = "rad", start = 0)) annotation(
        Placement(transformation(extent = {{-26, 88}, {-6, 108}})));
      Modelica.Blocks.Sources.Constant const(k = ampl) annotation(
        Placement(transformation(extent = {{-60, 138}, {-40, 158}})));
      Modelica.Blocks.Sources.SineVariableFrequencyAndAmplitude sine2(phi(fixed = true, start = pi/180*(-120), displayUnit = "rad")) annotation(
        Placement(transformation(extent = {{-2, 56}, {18, 76}})));
      Modelica.Blocks.Sources.SineVariableFrequencyAndAmplitude sine3(phi(fixed = true, start = pi/180*120, displayUnit = "rad")) annotation(
        Placement(transformation(extent = {{-24, 8}, {-4, 28}})));
      Modelica.Blocks.Sources.Sine sine15[3](amplitude = fill(ampl15, 3), f = fill(15*f2, 3), phase = pi/180*{0, -120, 120}, startTime = fill(startHTime, 3)) annotation(
        Placement(transformation(origin = {92, 90}, extent = {{-38, 64}, {-18, 84}})));
      Modelica.Blocks.Sources.Sine sine17[3](amplitude = fill(ampl17, 3), f = fill(17*f2, 3), phase = pi/180*{0, -120, 120}, startTime = fill(startHTime, 3)) annotation(
        Placement(transformation(origin = {58, 100}, extent = {{-36, 26}, {-16, 46}})));
      Modelica.Blocks.Math.Add3 add3[3] annotation(
        Placement(transformation(origin = {50, 74}, extent = {{10, 12}, {30, 32}}, rotation = 270)));
      Modelica.Electrical.Analog.Interfaces.NegativePin pin annotation(
        Placement(transformation(extent = {{-10, -50}, {10, -30}}), iconTransformation(extent = {{-10, -110}, {10, -90}})));
      Modelica.Electrical.Polyphase.Basic.Resistor Rtr(R = R*ones(3)) "transformer 25 MVA" annotation(
        Placement(transformation(extent = {{-72, -10}, {-52, 10}})));
      Modelica.Electrical.Polyphase.Basic.Inductor Ltr(L = L*ones(3)) "dati da Cagliari, trasformatore 25 MVA" annotation(
        Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug annotation(
        Placement(transformation(extent = {{-10, 168}, {10, 188}}), iconTransformation(extent = {{-8, 88}, {12, 108}})));
      Modelica.Blocks.Sources.Ramp ramp1(height = ampl2 - ampl, duration = rampV2duration, offset = ampl, startTime = startV2Time) annotation(
        Placement(transformation(extent = {{-88, 118}, {-68, 138}})));
    equation
      connect(signalVoltage.plug_n, star1.plug_p) annotation(
        Line(points = {{40, 0}, {46, 0}, {46, 0}, {62, 0}}, color = {0, 0, 255}));
      connect(ramp.y, sine1.f) annotation(
        Line(points = {{-69, 92}, {-28, 92}}, color = {0, 0, 127}));
      connect(ramp.y, sine3.f) annotation(
        Line(points = {{-69, 92}, {-44, 92}, {-44, 12}, {-26, 12}}, color = {0, 0, 127}));
      connect(sine2.f, ramp.y) annotation(
        Line(points = {{-4, 60}, {-44, 60}, {-44, 92}, {-69, 92}}, color = {0, 0, 127}));
      connect(sine17.y, add3.u2) annotation(
        Line(points = {{43, 136}, {72, 136}, {72, 66}}, color = {0, 0, 127}));
      connect(sine15.y, add3.u1) annotation(
        Line(points = {{75, 164}, {80, 164}, {80, 66}}, color = {0, 0, 127}));
      connect(add3.y, signalVoltage.v) annotation(
        Line(points = {{72, 43}, {72, 12}, {30, 12}}, color = {0, 0, 127}));
      connect(sine1.y, add3[1].u3) annotation(
        Line(points = {{-5, 98}, {40, 98}, {40, 66}, {64, 66}}, color = {0, 0, 127}));
      connect(sine2.y, add3[2].u3) annotation(
        Line(points = {{19, 66}, {64, 66}}, color = {0, 0, 127}));
      connect(sine3.y, add3[3].u3) annotation(
        Line(points = {{-3, 18}, {40, 18}, {40, 66}, {64, 66}}, color = {0, 0, 127}));
      connect(star1.pin_n, pin) annotation(
        Line(points = {{62, -20}, {0, -20}, {0, -40}}, color = {0, 0, 255}));
      connect(Rtr.plug_p, plug) annotation(
        Line(points = {{-72, 0}, {-96, 0}, {-96, 164}, {0, 164}, {0, 178}}, color = {0, 0, 255}));
      connect(Ltr.plug_p, Rtr.plug_n) annotation(
        Line(points = {{-40, 0}, {-52, 0}}, color = {0, 0, 255}));
      connect(Ltr.plug_n, signalVoltage.plug_p) annotation(
        Line(points = {{-20, 0}, {20, 0}}, color = {0, 0, 255}));
      connect(ramp1.y, sine1.amplitude) annotation(
        Line(points = {{-67, 128}, {-36, 128}, {-36, 104}, {-28, 104}}, color = {0, 0, 127}));
      connect(ramp1.y, sine2.amplitude) annotation(
        Line(points = {{-67, 128}, {-36, 128}, {-36, 72}, {-4, 72}}, color = {0, 0, 127}));
      connect(ramp1.y, sine3.amplitude) annotation(
        Line(points = {{-67, 128}, {-36, 128}, {-36, 24}, {-26, 24}}, color = {0, 0, 127}));
      annotation(
        experiment(StopTime = 3, Interval = 4e-05, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
        Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", origin = {130, 2}, rotation = 90), Ellipse(lineThickness = 0.5, extent = {{-46, 32}, {50, -58}}), Line(points = {{-56, -70}, {76, 38}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}), Rectangle(fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-30, 2}, {38, -36}}), Polygon(fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{66, 37}, {74, 28}, {82, 43}, {66, 37}}), Line(points = {{-32, -22}, {-26, -10}, {-20, -6}, {-16, -8}, {-12, -16}, {-10, -22}, {-6, -28}, {2, -30}, {8, -22}, {12, -12}, {15.1582, -4.63086}, {18, 2}, {22, 4}, {26, 0}, {28, -6}, {30, -14}, {32, -22}, {32, -26}}, color = {0, 0, 0}, thickness = 0.5, arrow = {Arrow.None, Arrow.None}), Line(points = {{2, 88}, {2, 32}}, color = {0, 0, 0}, thickness = 0.5, arrow = {Arrow.None, Arrow.None}), Rectangle(extent = {{-7, 78}, {11, 44}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
        Diagram(coordinateSystem(extent = {{-100, -40}, {100, 180}})),
        Documentation(info = "<html>
<p>genera una tensione di rete con </p>
<p>- una quota che modifica la sua frequenza da F1 a F2 , </p>
<p>- una quota di 15a armonica</p>
<p>- una quota di 17a armonica</p>
</html>"));
    end DistortedGrid;

    model GflBesC
      import Modelica.Constants.pi;
      parameter Modelica.Units.SI.Frequency fNom = 50 "Nominal frequency";
      parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Nominal apparent power";
      parameter Modelica.Units.SI.Voltage uDc = 750 "Nominal DC voltage";
      parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph line-to-line voltage (RMS), used for per-unit";
      parameter Modelica.Units.SI.Resistance rf = 0.00314 "Filter resistance ";
      parameter Modelica.Units.SI.Inductance lf = 5e-4 "Filter inductance ";
      parameter Modelica.Units.SI.Capacitance cf = 6e-6 "Filter capacitance";
      parameter Modelica.Units.SI.Resistance rcf = 2.56 "Filter resistance to C";
      parameter Modelica.Units.SI.Time delay = 0.0001 "Inverter time constant";
      parameter Modelica.Units.SI.Time delaycontrol = 0.0001 "GFL control id iq first order";
      parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom "Nominal Impedance";
      parameter Modelica.Units.SI.Reactance x = 2*pi*fNom*lf/zNom "VSC Reactance (PU)";
      parameter Modelica.Units.SI.Time tau_i = 1/(4*2*pi*fNom) "Time constant of current loop";
      parameter Modelica.Units.SI.Time tau_v = 1/(4*2*pi*fNom) "Time constant of feedforward vd e vq";
      parameter Real kp_i = x/tau_i/100/pi "Proportional gain of PI current regulators";
      parameter Real ki_i = kp_i/(0.05*100*pi) "Integral gain of PI current regulators";
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_ac annotation(
        Placement(transformation(origin = {100, -62}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {98, -60}, extent = {{-10, -10}, {10, 10}})));
      Inv inv(delay = delay) annotation(
        Placement(transformation(extent = {{-36, 12}, {-16, 32}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage uDcFem(V = uDc) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-78, 24})));
      Modelica.Electrical.Polyphase.Sensors.PotentialSensor vscVolt annotation(
        Placement(transformation(extent = {{8, -8}, {-8, 8}}, rotation = 270, origin = {74, 44})));
      Modelica.Blocks.Sources.RealExpression uDc_(y = uDcFem.v) annotation(
        Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 180, origin = {-80, -76})));
      Modelica.Electrical.Analog.Sensors.PowerSensor pDC annotation(
        Placement(transformation(extent = {{-66, 20}, {-46, 40}})));
      Modelica.Electrical.Polyphase.Sensors.CurrentSensor vscCurr annotation(
        Placement(transformation(origin = {86, 28}, extent = {{-8, 8}, {8, -8}})));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug annotation(
        Placement(transformation(origin = {100, 28}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {102, 58}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = rf*ones(3)) annotation(
        Placement(transformation(extent = {{22, 18}, {42, 38}})));
      Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = lf*ones(3)) annotation(
        Placement(transformation(extent = {{48, 18}, {68, 38}})));
      Modelica.Blocks.Interfaces.RealInput pRef annotation(
        Placement(transformation(origin = {-140, 22}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput qRef annotation(
        Placement(transformation(origin = {-140, -46}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Blocks.Gflcontrol GFLcontrol(fNom = fNom, sNom = sNom, uAcNom = uAcNom, rf = rf, lf = lf, cf = cf, delaycontrol = delaycontrol, zNom = zNom, tau_i = tau_i, x = x, tau_v = tau_v, kp_i = kp_i, ki_i = ki_i) annotation(
        Placement(transformation(extent = {{-82, -62}, {-38, -14}})));
      Blocks.Park.DqToAbc dqToAbc annotation(
        Placement(transformation(extent = {{10, -76}, {30, -56}})));
      Blocks.PLLdedicated.PLL pLL(maxSlew = 5) annotation(
        Placement(transformation(extent = {{28, 68}, {48, 88}})));
      Modelica.Blocks.Sources.RealExpression pLLtheta(y = pLL.thetaRad) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {62, -90})));
      Modelica.Blocks.Math.Gain toVpu[3](k = (1/(uAcNom/sqrt(3)*sqrt(2)))*ones(3)) annotation(
        Placement(transformation(origin = {-6, 78}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Electrical.Polyphase.Basic.Capacitor capacitor(C = cf*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {74, -22})));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {74, -48})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistorC(R = rcf*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {74, 6})));
      Modelica.Blocks.Logical.Switch switch1[2] annotation(
        Placement(transformation(extent = {{-8, -8}, {8, 8}}, rotation = 0, origin = {-12, -44})));
      Modelica.Blocks.Sources.Constant vdqnom[2](k = {(uAcNom/sqrt(3)*sqrt(2)), 0}) annotation(
        Placement(transformation(extent = {{-42, -82}, {-32, -72}})));
      Modelica.Blocks.Interfaces.BooleanInput u[3] annotation(
        Placement(transformation(extent = {{-11, -11}, {11, 11}}, rotation = 90, origin = {5, -19}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {80, 120})));
      Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m = 3, Ron = fill(1e-5, 3), Goff = fill(1e-5, 3)) annotation(
        Placement(transformation(origin = {4, 28}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    equation
      connect(uDcFem.n, inv.pin_n) annotation(
        Line(points = {{-78, 14}, {-78, 8}, {-56, 8}, {-56, 15.8}, {-36, 15.8}}, color = {0, 0, 255}));
      connect(pDC.nv, uDcFem.n) annotation(
        Line(points = {{-56, 20}, {-56, 8}, {-78, 8}, {-78, 14}}, color = {0, 0, 255}));
      connect(pDC.pv, pDC.nc) annotation(
        Line(points = {{-56, 40}, {-46, 40}, {-46, 30}}, color = {0, 0, 255}));
      connect(pDC.pc, uDcFem.p) annotation(
        Line(points = {{-66, 30}, {-66, 34}, {-78, 34}}, color = {0, 0, 255}));
      connect(pDC.nc, inv.pin_p) annotation(
        Line(points = {{-46, 30}, {-42, 30}, {-42, 28}, {-36, 28}}, color = {0, 0, 255}));
      connect(plug, vscCurr.plug_n) annotation(
        Line(points = {{100, 28}, {94, 28}}, color = {0, 0, 255}));
      connect(inv.pin_ac, pin_ac) annotation(
        Line(points = {{-16.2, 16}, {-8, 16}, {-8, 8}, {54, 8}, {54, -62}, {100, -62}}, color = {0, 0, 255}));
      connect(resistor.plug_n, inductor.plug_p) annotation(
        Line(points = {{42, 28}, {48, 28}}, color = {0, 0, 255}));
      connect(inv.pin_n, pin_ac) annotation(
        Line(points = {{-36, 15.8}, {-46, 15.8}, {-46, 2}, {-8, 2}, {-8, 8}, {54, 8}, {54, -62}, {100, -62}}, color = {0, 0, 255}));
      connect(qRef, GFLcontrol.qref) annotation(
        Line(points = {{-140, -46}, {-140, -47.6}, {-86.4, -47.6}}, color = {0, 0, 127}));
      connect(pRef, GFLcontrol.pref) annotation(
        Line(points = {{-140, 22}, {-104, 22}, {-104, -38}, {-86.4, -38}}, color = {0, 0, 127}));
      connect(vscVolt.phi, GFLcontrol.vabc) annotation(
        Line(points = {{74, 52.8}, {74, 58}, {-94, 58}, {-94, -18.8}, {-86.4, -18.8}}, color = {0, 0, 127}));
      connect(uDc_.y, GFLcontrol.uDc) annotation(
        Line(points = {{-69, -76}, {-55.6, -76}, {-55.6, -66.8}}, color = {0, 0, 127}));
      connect(pLLtheta.y, dqToAbc.theta) annotation(
        Line(points = {{51, -90}, {2, -90}, {2, -71.6}, {10.8, -71.6}}, color = {0, 0, 127}));
      connect(GFLcontrol.theta, pLLtheta.y) annotation(
        Line(points = {{-86.4, -57.2}, {-94, -57.2}, {-94, -90}, {51, -90}}, color = {0, 0, 127}));
      connect(toVpu.u, vscVolt.phi) annotation(
        Line(points = {{-18, 78}, {-26, 78}, {-26, 58}, {74, 58}, {74, 52.8}}, color = {0, 0, 127}));
      connect(toVpu.y, pLL.u) annotation(
        Line(points = {{5, 78}, {26, 78}}, color = {0, 0, 127}));
      connect(capacitor.plug_n, star.plug_p) annotation(
        Line(points = {{74, -32}, {74, -38}}, color = {0, 0, 255}));
      connect(star.pin_n, pin_ac) annotation(
        Line(points = {{74, -58}, {74, -62}, {100, -62}}, color = {0, 0, 255}));
      connect(capacitor.plug_p, resistorC.plug_n) annotation(
        Line(points = {{74, -12}, {74, -4}}, color = {0, 0, 255}));
      connect(vdqnom.y, switch1.u3) annotation(
        Line(points = {{-31.5, -77}, {-31.5, -78}, {-28, -78}, {-28, -50.4}, {-21.6, -50.4}}, color = {0, 0, 127}));
      connect(dqToAbc.abc, inv.u) annotation(
        Line(points = {{30.2, -66}, {34, -66}, {34, -28}, {-22, -28}, {-22, 0}, {-26, 0}, {-26, 10.8}}, color = {0, 0, 127}));
      connect(GFLcontrol.mdqV, switch1.u1) annotation(
        Line(points = {{-35.8, -38}, {-36, -37.6}, {-21.6, -37.6}}, color = {0, 0, 127}));
      connect(resistorC.plug_p, vscCurr.plug_p) annotation(
        Line(points = {{74, 16}, {74, 28}, {78, 28}}, color = {0, 0, 255}));
      connect(vscVolt.plug_p, vscCurr.plug_p) annotation(
        Line(points = {{74, 36}, {74, 28}, {78, 28}}, color = {0, 0, 255}));
      connect(vscCurr.plug_p, inductor.plug_n) annotation(
        Line(points = {{78, 28}, {68, 28}}, color = {0, 0, 255}));
      connect(switch1.y, dqToAbc.dq) annotation(
        Line(points = {{-3.2, -44}, {2, -44}, {2, -62.8}, {10.8, -62.8}}, color = {0, 0, 127}));
      connect(GFLcontrol.iabc, vscCurr.i) annotation(
        Line(points = {{-86.4, -28.4}, {-98, -28.4}, {-98, 94}, {86, 94}, {86, 36.8}}, color = {0, 0, 127}));
      connect(u[1], switch1[1].u2) annotation(
        Line(points = {{5, -22.6667}, {-28, -22.6667}, {-28, -44}, {-21.6, -44}}, color = {255, 0, 255}));
      connect(u[2], switch1[2].u2) annotation(
        Line(points = {{5, -19}, {4, -19}, {4, -22}, {-28, -22}, {-28, -44}, {-21.6, -44}}, color = {255, 0, 255}));
      connect(resistor.plug_p, idealCloser.plug_n) annotation(
        Line(points = {{22, 28}, {14, 28}}, color = {0, 0, 255}));
      connect(inv.plug, idealCloser.plug_p) annotation(
        Line(points = {{-16, 28}, {-6, 28}}, color = {0, 0, 255}));
      connect(u, idealCloser.control) annotation(
        Line(points = {{5, -19}, {4, -19}, {4, 16}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-106, 146}, {104, 104}}, textString = "%name"), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-96, 70}, {-30, 52}}, textString = "pRef"), Text(origin = {2, 0}, extent = {{-96, -50}, {-30, -68}}, textString = "qRef"), Line(points = {{-86, 14}, {-30, 14}}), Rectangle(extent = {{-24, 84}, {74, -80}}, lineColor = {28, 108, 200}), Line(points = {{-24, -80}, {74, 84}}, color = {28, 108, 200}), Line(points = {{10, -54}, {20, -36}, {30, -36}, {44, -68}, {54, -68}, {62, -50}}), Line(points = {{92, 58}, {74, 58}}, color = {28, 108, 200}), Line(points = {{90, -60}, {74, -60}}, color = {28, 108, 200}), Line(points = {{-24, 42}, {-58, 42}, {-58, 14}}, color = {28, 108, 200}), Line(points = {{-24, -38}, {-58, -38}, {-58, -10}}, color = {28, 108, 200}), Line(points = {{-8, 60}, {34, 60}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{-72, 0}, {-44, -12}}), Text(extent = {{-34, 30}, {84, -28}}, textColor = {0, 0, 0}, textString = "GFL")}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-120, -100}, {100, 100}})),
        Documentation(info = "<html>
<p>Models a, AC-interfaced Battery energy storage system.</p>
<p>It contains a Constant DC source, and a Voltage Sourced Converter (VSC), and its control</p>
<p>The VSC is simulated generating its fundamental-frequency component three-phase balanced set of electromotive forces, which feeds an R-L-C filter.</p>
<p>The control system is GFL...</p>
<p>The internal structure of this component is organised in a way that favours new models to be easily built, e.g. with a richer DC-side network or control logic </p>
</html>"));
    end GflBesC;

    model GfmDroopBesC
      import Modelica.Constants.pi;
      parameter Modelica.Units.SI.Frequency fNom = 50 "Nominal frequency";
      parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Nominal apparent power";
      parameter Modelica.Units.SI.Voltage uDc = 750 "Nominal DC voltage (used for per-unit)";
      parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph voltage (used for per-unit)";
      parameter Modelica.Units.SI.Resistance rf = 0.00314 "Filter resistance";
      parameter Modelica.Units.SI.Inductance lf = 5e-4 "Filter inductance";
      parameter Modelica.Units.SI.Capacitance cf = 6e-6 "Filter capacitance";
      parameter Modelica.Units.SI.Resistance rcf = 2.56 "Filter resistance to C";
      parameter Modelica.Units.SI.Time delay = 0.0001 "Inverter time constant";
      parameter Real pGain = 0.05 "P-loop control gain (PU)";
      parameter Real qGain = 0.05 "Q-loop control gain (PU)";
      parameter Modelica.Units.SI.Time fo = 0.02 "First order time constant for P and Q calculation in VSC control";
      parameter Real xcc = 0.058 "Xcc pu transfomer";
      parameter Real secReg = 1 "Secondary regulation, 1=ON 0=OFF";
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_ac annotation(
        Placement(transformation(origin = {100, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {98, -60}, extent = {{-10, -10}, {10, 10}})));
      Inv inv(delay = delay) annotation(
        Placement(transformation(extent = {{-36, 12}, {-16, 32}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage uDcFem(V = uDc) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-78, 26})));
      Blocks.GfmDroopControl GFDControl(fNom = fNom, sNom = sNom, uDc = uDc, uAcNom = uAcNom, pGain = pGain, qGain = qGain, fo = fo, xcc = xcc, secReg = secReg) annotation(
        Placement(transformation(extent = {{-86, -52}, {-44, -10}})));
      Blocks.Park.DToAbc dToAbc annotation(
        Placement(transformation(extent = {{-28, -38}, {-8, -18}})));
      Modelica.Electrical.Polyphase.Sensors.PotentialSensor vscVolt annotation(
        Placement(transformation(extent = {{8, -8}, {-8, 8}}, rotation = 270, origin = {50, 46})));
      Modelica.Blocks.Sources.RealExpression uDcMeas(y = uDcFem.v) annotation(
        Placement(transformation(extent = {{-22, -78}, {-42, -58}})));
      Modelica.Electrical.Analog.Sensors.PowerSensor pDC annotation(
        Placement(transformation(extent = {{-66, 20}, {-46, 40}})));
      Modelica.Electrical.Polyphase.Sensors.CurrentSensor vscCurr annotation(
        Placement(transformation(origin = {72, 28}, extent = {{-8, 8}, {8, -8}})));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug annotation(
        Placement(transformation(origin = {100, 28}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {102, 58}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = rf*ones(3)) annotation(
        Placement(transformation(extent = {{-6, 18}, {14, 38}})));
      Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = lf*ones(3)) annotation(
        Placement(transformation(extent = {{22, 18}, {42, 38}})));
      Modelica.Blocks.Interfaces.RealInput pRef annotation(
        Placement(transformation(origin = {-140, -18}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput qRef annotation(
        Placement(transformation(origin = {-140, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Capacitor capacitor(C = cf*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {50, -12})));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {50, -36})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistorC(R = rcf*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {50, 12})));
    equation
      connect(uDcFem.n, inv.pin_n) annotation(
        Line(points = {{-78, 16}, {-78, 8}, {-56, 8}, {-56, 15.8}, {-36, 15.8}}, color = {0, 0, 255}));
      connect(GFDControl.mdV, dToAbc.d) annotation(
        Line(points = {{-41.9, -18.4}, {-41.9, -22}, {-27.6, -22}}, color = {0, 0, 127}));
      connect(GFDControl.theta, dToAbc.theta) annotation(
        Line(points = {{-41.9, -41.92}, {-40, -41.92}, {-40, -33.6}, {-27.2, -33.6}}, color = {0, 0, 127}));
      connect(uDcMeas.y, GFDControl.uDcMeas) annotation(
        Line(points = {{-43, -68}, {-62, -68}, {-62, -56.2}, {-60.8, -56.2}}, color = {0, 0, 127}));
      connect(dToAbc.abc, inv.u) annotation(
        Line(points = {{-7.8, -28}, {0, -28}, {0, -10}, {-26, -10}, {-26, 10.8}}, color = {0, 0, 127}));
      connect(pDC.nv, uDcFem.n) annotation(
        Line(points = {{-56, 20}, {-56, 8}, {-78, 8}, {-78, 16}}, color = {0, 0, 255}));
      connect(pDC.pv, pDC.nc) annotation(
        Line(points = {{-56, 40}, {-46, 40}, {-46, 30}}, color = {0, 0, 255}));
      connect(pDC.pc, uDcFem.p) annotation(
        Line(points = {{-66, 30}, {-66, 36}, {-78, 36}}, color = {0, 0, 255}));
      connect(pDC.nc, inv.pin_p) annotation(
        Line(points = {{-46, 30}, {-42, 30}, {-42, 28}, {-36, 28}}, color = {0, 0, 255}));
      connect(GFDControl.vabc, vscVolt.phi) annotation(
        Line(points = {{-90.2, -14.2}, {-100, -14.2}, {-100, 60}, {50, 60}, {50, 54.8}}, color = {0, 0, 127}));
      connect(plug, vscCurr.plug_n) annotation(
        Line(points = {{100, 28}, {80, 28}}, color = {0, 0, 255}));
      connect(vscCurr.i, GFDControl.iabc) annotation(
        Line(points = {{72, 36.8}, {72, 66}, {-104, 66}, {-104, -25.12}, {-90.2, -25.12}}, color = {0, 0, 127}));
      connect(GFDControl.qRef, qRef) annotation(
        Line(points = {{-90.2, -47.8}, {-92, -48}, {-140, -48}}, color = {0, 0, 127}));
      connect(pRef, GFDControl.pRef) annotation(
        Line(points = {{-140, -18}, {-108, -18}, {-108, -36.88}, {-90.2, -36.88}}, color = {0, 0, 127}));
      connect(inv.pin_ac, pin_ac) annotation(
        Line(points = {{-16.2, 16}, {-10, 16}, {-10, 0}, {18, 0}, {18, -50}, {100, -50}}, color = {0, 0, 255}));
      connect(vscCurr.plug_p, inductor.plug_n) annotation(
        Line(points = {{64, 28}, {42, 28}}, color = {0, 0, 255}));
      connect(resistor.plug_n, inductor.plug_p) annotation(
        Line(points = {{14, 28}, {22, 28}}, color = {0, 0, 255}));
      connect(vscVolt.plug_p, inductor.plug_n) annotation(
        Line(points = {{50, 38}, {50, 28}, {42, 28}}, color = {0, 0, 255}));
      connect(inv.plug, resistor.plug_p) annotation(
        Line(points = {{-16, 28}, {-6, 28}}, color = {0, 0, 255}));
      connect(inv.pin_n, pin_ac) annotation(
        Line(points = {{-36, 15.8}, {-40, 15.8}, {-40, 0}, {18, 0}, {18, -50}, {100, -50}}, color = {0, 0, 255}));
      connect(capacitor.plug_n, star.plug_p) annotation(
        Line(points = {{50, -22}, {50, -26}}, color = {0, 0, 255}));
      connect(star.pin_n, pin_ac) annotation(
        Line(points = {{50, -46}, {50, -50}, {100, -50}}, color = {0, 0, 255}));
      connect(capacitor.plug_p, resistorC.plug_n) annotation(
        Line(points = {{50, -2}, {50, 2}}, color = {0, 0, 255}));
      connect(resistorC.plug_p, inductor.plug_n) annotation(
        Line(points = {{50, 22}, {50, 28}, {42, 28}}, color = {0, 0, 255}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-106, 146}, {104, 104}}, textString = "%name"), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(extent = {{-94, 70}, {-28, 52}}, textString = "pRef"), Text(extent = {{-94, -50}, {-28, -68}}, textString = "qRef"), Line(points = {{-86, 14}, {-30, 14}}), Rectangle(extent = {{-24, 84}, {74, -80}}, lineColor = {28, 108, 200}), Line(points = {{-24, -80}, {74, 84}}, color = {28, 108, 200}), Line(points = {{10, -54}, {20, -36}, {30, -36}, {44, -68}, {54, -68}, {62, -50}}), Line(points = {{92, 58}, {74, 58}}, color = {28, 108, 200}), Line(points = {{90, -60}, {74, -60}}, color = {28, 108, 200}), Line(points = {{-24, 42}, {-58, 42}, {-58, 14}}, color = {28, 108, 200}), Line(points = {{-24, -38}, {-58, -38}, {-58, -10}}, color = {28, 108, 200}), Line(points = {{-8, 60}, {34, 60}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{-72, 0}, {-44, -12}}), Text(extent = {{-34, 30}, {84, -28}}, textColor = {0, 0, 0}, textString = "GFD")}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-120, -80}, {100, 80}})),
        Documentation(info = "<html>
<p>Models a, AC-interfaced Battery energy storage system.</p>
<p>It contains a Constant DC source, and a Voltage Sourced Converter (VSC), and its control</p>
<p>The VSC is simulated generating its fundamental-frequency component three-phase balanced set of electromotive forces, which feeds an R-L-C filter.</p>
<p>The control system is DROOP GFM<b><span style=\"color: #ff0000;\"> (da scegliere un acronimo per questo modello con statismo da riportare anche nell&apos;icona)</span></b>...</p>
<p>The internal structure of this component is organised in a way that favours new models to be easily built, e.g. with a richer DC-side network or control logic </p>
</html>"));
    end GfmDroopBesC;

    model ImScScalable "induction motor 3ph squirrel cage, insertion with desired initial speed, scalable with Apparent Power and Voltage"
      constant Integer m = 3 "Number of phases";
      import Modelica.Constants.pi;
      parameter Modelica.Units.SI.Voltage VNominal = 100 "Nominal RMS voltage per phase, used for scaling (100 V original -> 400/sqrt(3) )";
      parameter Modelica.Units.SI.Frequency fNominal = 50 "Nominal frequency";
      parameter Modelica.Units.SI.Time tStart1 = 1 "Start time";
      parameter Modelica.Units.SI.Torque TLoad = Pnet/wLoad "Nominal load torque (161.4 original -> Pnet/wLoad";
      parameter Modelica.Units.SI.AngularVelocity wLoad = 1440.45*2*Modelica.Constants.pi/60 "Nominal load speed";
      parameter Modelica.Units.SI.AngularVelocity wInit = 1440.45*2*Modelica.Constants.pi/60 "Initial speed = Nominal load speed";
      parameter Modelica.Units.SI.Inertia JLoad = (2*HO*Sbase)/((2*pi*fNominal/aimc.p)^2) "Load's moment of inertia, Jr, 0.29 original -> (2*HO*Sbase)/((2*pi*fNominal/aimc.p)^2)";
      parameter Modelica.Units.SI.ApparentPower Sbase = 10000 "Apparent power, used for scaling (30000 VA original)";
      final parameter Modelica.Units.SI.Voltage Vbase = VNominal*sqrt(2) "Vbase";
      final parameter Modelica.Units.SI.Current Ibase = Sbase/Vbase*2/3 "Ibase";
      final parameter Modelica.Units.SI.Impedance Zbase = Vbase/Ibase "Zbase";
      final parameter Modelica.Units.SI.Inductance Lbase = Zbase/(2*pi*fNominal) "Lbase";
      final parameter Modelica.Units.SI.Power Pnet = Sbase*PFO*EffO "Output power calculated with original efficiency and power factor";
      //base values of original model, modified with Y connection, Modelica.Electrical.Machines.Examples.InductionMachines.IMC_DOL
      final parameter Modelica.Units.SI.Voltage VBO = 100*sqrt(2) "Vbase original model";
      final parameter Modelica.Units.SI.Current IBO = 100*sqrt(2) "Ibase original model";
      final parameter Modelica.Units.SI.ApparentPower SBO = 3/2*VBO*IBO "Sbase original model";
      final parameter Modelica.Units.SI.Impedance ZBO = VBO/IBO "Zbase original model";
      final parameter Modelica.Units.SI.Time HO = 0.29*((2*pi*50/2)^2)/(2*SBO) "time constant original model";
      final parameter Modelica.Units.SI.Inductance LBO = ZBO/(2*pi*50) "Lbase original model";
      final parameter Modelica.Units.SI.PowerFactor PFO = 0.875 "Power factor original model";
      final parameter Modelica.Units.SI.Efficiency EffO = 24346/(SBO*PFO) "efficiency original model, 24.346kW, 30000VA, 0.875 power factor";
      //scaled parameters for aimcData
      final parameter Modelica.Units.SI.Resistance RsScaled = 0.03/ZBO*Zbase;
      final parameter Modelica.Units.SI.Inductance LszeroScaled = LssigmaScaled;
      final parameter Modelica.Units.SI.Inductance LssigmaScaled = (3*(1 - sqrt(1 - 0.0667))/(2*pi*50))/LBO*Lbase;
      final parameter Modelica.Units.SI.Inductance LmScaled = (3*sqrt(1 - 0.0667)/(2*pi*50))/LBO*Lbase;
      final parameter Modelica.Units.SI.Inductance LrsigmaScaled = (3*(1 - sqrt(1 - 0.0667))/(2*pi*50))/LBO*Lbase;
      final parameter Modelica.Units.SI.Resistance RrScaled = 0.04/ZBO*Zbase;
      Modelica.Electrical.Machines.BasicMachines.InductionMachines.IM_SquirrelCage aimc(p = aimcData.p, fsNominal = aimcData.fsNominal, Rs = aimcData.Rs, TsRef = aimcData.TsRef, alpha20s(displayUnit = "1/K") = aimcData.alpha20s, Lszero = aimcData.Lszero, Lssigma = aimcData.Lssigma, Jr = aimcData.Jr, Js = aimcData.Js, frictionParameters = aimcData.frictionParameters, phiMechanical(fixed = false), wMechanical(fixed = true, start = wInit), statorCoreParameters = aimcData.statorCoreParameters, strayLoadParameters = aimcData.strayLoadParameters, Lm = aimcData.Lm, Lrsigma = aimcData.Lrsigma, Rr = aimcData.Rr, TrRef = aimcData.TrRef, TsOperational = 293.15, alpha20r = aimcData.alpha20r, TrOperational = 293.15) annotation(
        Placement(transformation(extent = {{-80, -16}, {-60, 4}})));
      Modelica.Blocks.Sources.BooleanStep booleanStep[m](each startTime = tStart1) annotation(
        Placement(transformation(extent = {{90, 36}, {70, 56}})));
      Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m = m, Ron = fill(1e-5, m), Goff = fill(1e-5, m)) annotation(
        Placement(transformation(origin = {-8, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J = JLoad) annotation(
        Placement(transformation(extent = {{-46, -16}, {-26, 4}})));
      Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticLoadTorque(w_nominal = wLoad, TorqueDirection = false, tau_nominal = -TLoad, useSupport = false) annotation(
        Placement(transformation(extent = {{46, 4}, {26, 24}})));
      Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(terminalConnection = "Y") annotation(
        Placement(transformation(extent = {{-80, 0}, {-60, 20}})));
      parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.IM_SquirrelCageData aimcData(Jr = (2*HO*Sbase)/((2*pi*fNominal/aimc.p)^2), Rs = RsScaled, Lszero = LszeroScaled, Lssigma = LssigmaScaled, Lm = LmScaled, Lrsigma = LrsigmaScaled, Rr = RrScaled) "Induction machine data" annotation(
        Placement(transformation(extent = {{-92, 40}, {-72, 60}})));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {-54, -36})));
      Modelica.Mechanics.Rotational.Sources.Torque torque "torque to mantain initial speed" annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {8, -30})));
      Modelica.Blocks.Math.Feedback feedback annotation(
        Placement(transformation(extent = {{-64, -60}, {-44, -80}})));
      Modelica.Blocks.Sources.Constant const(k = wInit) annotation(
        Placement(transformation(extent = {{-88, -78}, {-72, -62}})));
      Modelica.Blocks.Math.Gain gain(k = 100) annotation(
        Placement(transformation(extent = {{-14, -68}, {2, -52}})));
      Modelica.Blocks.Logical.Switch switch1 annotation(
        Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = 180, origin = {40, -30})));
      Modelica.Blocks.Sources.Constant const1(k = 0) annotation(
        Placement(transformation(extent = {{-8, -8}, {8, 8}}, rotation = 180, origin = {88, -22})));
      Modelica.Blocks.Math.Add add annotation(
        Placement(transformation(extent = {{24, -80}, {40, -64}})));
      Modelica.Blocks.Sources.Constant const2(k = TLoad) annotation(
        Placement(transformation(extent = {{-8, -8}, {8, 8}}, rotation = 0, origin = {-10, -86})));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug annotation(
        Placement(transformation(extent = {{50, 90}, {70, 110}}), iconTransformation(extent = {{90, 50}, {110, 70}})));
    initial equation
      aimc.is = zeros(3);
      aimc.ir = zeros(2);
    equation
      connect(booleanStep.y, idealCloser.control) annotation(
        Line(points = {{69, 46}, {12, 46}, {12, 58}, {4, 58}}, color = {255, 0, 255}));
      connect(terminalBox.plug_sn, aimc.plug_sn) annotation(
        Line(points = {{-76, 4}, {-76, 4}}, color = {0, 0, 255}));
      connect(terminalBox.plug_sp, aimc.plug_sp) annotation(
        Line(points = {{-64, 4}, {-64, 4}}, color = {0, 0, 255}));
      connect(loadInertia.flange_b, quadraticLoadTorque.flange) annotation(
        Line(points = {{-26, -6}, {8, -6}, {8, 14}, {26, 14}}));
      connect(aimc.flange, loadInertia.flange_a) annotation(
        Line(points = {{-60, -6}, {-46, -6}}));
      connect(torque.flange, loadInertia.flange_b) annotation(
        Line(points = {{-2, -30}, {-14, -30}, {-14, -6}, {-26, -6}}, color = {0, 0, 0}));
      connect(speedSensor.flange, aimc.flange) annotation(
        Line(points = {{-54, -26}, {-54, -6}, {-60, -6}}, color = {0, 0, 0}));
      connect(switch1.y, torque.tau) annotation(
        Line(points = {{29, -30}, {20, -30}}, color = {0, 0, 127}));
      connect(booleanStep[1].y, switch1.u2) annotation(
        Line(points = {{69, 46}, {62, 46}, {62, -30}, {52, -30}}, color = {255, 0, 255}));
      connect(const.y, feedback.u1) annotation(
        Line(points = {{-71.2, -70}, {-62, -70}}, color = {0, 0, 127}));
      connect(speedSensor.w, feedback.u2) annotation(
        Line(points = {{-54, -47}, {-54, -62}}, color = {0, 0, 127}));
      connect(feedback.y, gain.u) annotation(
        Line(points = {{-45, -70}, {-24, -70}, {-24, -60}, {-15.6, -60}}, color = {0, 0, 127}));
      connect(const1.y, switch1.u1) annotation(
        Line(points = {{79.2, -22}, {52, -22}}, color = {0, 0, 127}));
      connect(gain.y, add.u1) annotation(
        Line(points = {{2.8, -60}, {18, -60}, {18, -67.2}, {22.4, -67.2}}, color = {0, 0, 127}));
      connect(const2.y, add.u2) annotation(
        Line(points = {{-1.2, -86}, {16, -86}, {16, -76.8}, {22.4, -76.8}}, color = {0, 0, 127}));
      connect(add.y, switch1.u3) annotation(
        Line(points = {{40.8, -72}, {62, -72}, {62, -38}, {52, -38}}, color = {0, 0, 127}));
      connect(idealCloser.plug_p, plug) annotation(
        Line(points = {{-8, 68}, {-8, 86}, {60, 86}, {60, 100}}, color = {0, 0, 255}));
      connect(idealCloser.plug_n, terminalBox.plugSupply) annotation(
        Line(points = {{-8, 48}, {-8, 20}, {-70, 20}, {-70, 6}}, color = {0, 0, 255}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {6, 0}, extent = {{-120, 28}, {114, -32}}, textString = "Induction
Motor
sq.cage")}),
        Diagram(coordinateSystem(preserveAspectRatio = false)));
    end ImScScalable;

    model LoadRL
      import Modelica.Constants.pi;
      parameter Real cosfi = 0.9 "Power factor, cosfi";
      final parameter Real fi = acos(cosfi) "angle fi";
      final parameter Real xr = tan(fi) "x/r ratio";
      parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
      parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph line-to-line voltage (RMS)";
      final parameter Modelica.Units.SI.Voltage vAcNom = uAcNom/sqrt(3) "Nominal AC 3ph line-to-neutral voltage (RMS)";
      parameter Modelica.Units.SI.ActivePower pNom = 100e3 "Nominal active power";
      final parameter Modelica.Units.SI.Current iAcNom = pNom/(sqrt(3)*uAcNom*cosfi) "Val. nominale corrente di linea AC trifase rms";
      final parameter Modelica.Units.SI.Impedance zNom = vAcNom/iAcNom "Impedenza nominale, radq(r^2+x^2)";
      final parameter Modelica.Units.SI.Resistance r = sqrt(zNom^2/(1 + xr^2)) "resistenza";
      final parameter Modelica.Units.SI.Reactance x = r*xr "reattanza";
      final parameter Modelica.Units.SI.Inductance l = x/(2*pi*f) "induttanza";
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = r*ones(3)) annotation(
        Placement(transformation(extent = {{40, -10}, {60, 10}})));
      Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = l*ones(3)) annotation(
        Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug positivePlug annotation(
        Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
      Modelica.Electrical.Polyphase.Interfaces.NegativePlug negativePlug annotation(
        Placement(transformation(extent = {{90, -10}, {110, 10}})));
    equation
      connect(resistor.plug_n, negativePlug) annotation(
        Line(points = {{60, 0}, {100, 0}}, color = {0, 0, 255}));
      connect(resistor.plug_p, inductor.plug_n) annotation(
        Line(points = {{40, 0}, {-20, 0}}, color = {0, 0, 255}));
      connect(inductor.plug_p, positivePlug) annotation(
        Line(points = {{-40, 0}, {-100, 0}}, color = {0, 0, 255}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-102, 88}, {98, 48}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-96, 16}, {94, -18}}, textColor = {0, 0, 0}, textString = "RL load 3ph")}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})));
    end LoadRL;

    package MV
      model GflBesCTransfDy "MV-LV transformer + GFLBES with C filter"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Modelica.Units.SI.Time tConn = -10 "Inverter connection time";
        parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Nominal apparent power";
        parameter Modelica.Units.SI.Voltage uDc = 750 "Nominal DC voltage";
        parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph line-to-line voltage (RMS), used for per-unit";
        parameter Real rfpu = lfpu/10 "Filter resistance (PU)";
        parameter Real lfpu = 0.05 "Filter inductance (PU)";
        parameter Real cfpu = 0.03 "Filter capacitance (PU)";
        parameter Real rcfpu = 1.6 "Filter resistance to C (PU)";
        final parameter Modelica.Units.SI.Resistance rf = rfpu*uAcNom*uAcNom/sNom "Filter resistance ";
        final parameter Modelica.Units.SI.Inductance lf = lfpu*uAcNom*uAcNom/(sNom*2*pi*f) "Filter inductance ";
        final parameter Modelica.Units.SI.Capacitance cf = cfpu*cNom "Filter capacitance";
        final parameter Modelica.Units.SI.Resistance rcf = rcfpu*zNom "Filter resistance to C";
        final parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom "Nominal impedance";
        final parameter Modelica.Units.SI.Capacitance cNom = 1/(zNom*2*pi*f) "Nominal capacitance";
        parameter Modelica.Units.SI.Time delay = 0.0001 "Inverter time constant";
        parameter Modelica.Units.SI.Voltage V1 = 20e3 "Transformer primary nominal line-to-line voltage (RMS)";
        parameter Modelica.Units.SI.Voltage V2 = 400 "Transformer secondary open circuit line-to-line voltage (RMS) @ primary nominal voltage";
        parameter Modelica.Units.SI.ApparentPower SNominal = 200e3 "Nominal apparent power transformer";
        parameter Real v_sc = 0.06 "Zcc (PU), Impedance voltage drop pu transformer";
        parameter Real rcc = 0.0015 "Rcc (PU) transfomer, copper losses";
        parameter Real xcc = sqrt(v_sc^2 - rcc^2) "Xcc (PU) transfomer";
        final parameter Modelica.Units.SI.Power P_sc = rcc*SNominal "Short-circuit (copper) losses transformer";
        final parameter Modelica.Units.SI.Impedance ZbTr = V2*V2/SNominal "Zbase transformer";
        final parameter Modelica.Units.SI.Inductance lg = (xcc*ZbTr)/(2*pi*f) "L transformer";
        Modelica.Electrical.Polyphase.Sensors.AronSensor pVsc annotation(
          Placement(transformation(origin = {52, 50}, extent = {{-92, -60}, {-72, -40}})));
        Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qVsc annotation(
          Placement(transformation(origin = {48, 50}, extent = {{-62, -60}, {-42, -40}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug plugIn "from MV grid" annotation(
          Placement(transformation(extent = {{-10, 90}, {10, 110}}), iconTransformation(extent = {{-10, 90}, {10, 110}})));
        parameter Modelica.Electrical.Machines.Utilities.TransformerData transformerData(C1 = "D", C2 = "y", f = f, V1 = V1, V2 = V2, SNominal = SNominal, v_sc = v_sc, P_sc = P_sc) "Transformer data" annotation(
          Placement(transformation(extent = {{60, 30}, {80, 50}})));
        Modelica.Electrical.Machines.BasicMachines.Transformers.Dy.Dy01 transformer(n = transformerData.n, R1 = transformerData.R1, L1sigma = transformerData.L1sigma, R2 = transformerData.R2, L2sigma = transformerData.L2sigma, T1Ref = 293.15, alpha20_1(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T2Ref = 293.15, alpha20_2(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T1Operational = 293.15, T2Operational = 293.15) annotation(
          Placement(transformation(extent = {{90, -20}, {50, 20}})));
        Modelica.Blocks.Interfaces.RealInput pqRef[2] annotation(
          Placement(transformation(origin = {-110, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -76}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep booleanStep[3](each startTime = tConn) annotation(
          Placement(transformation(extent = {{60, 60}, {40, 80}})));
        GflBesC gflBesC(fNom = f, sNom = sNom, uDc = uDc, uAcNom = uAcNom, rf = rf, lf = lf, cf = cf, rcf = rcf, delay = delay) annotation(
          Placement(transformation(extent = {{-66, -16}, {-46, 4}})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(origin = {2, -48}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.BooleanStep booleanStep1[3](each startTime = tConn + 0.5) annotation(
          Placement(transformation(extent = {{-22, 34}, {-42, 54}})));
        Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m = 3, Ron = fill(1e-5, 3), Goff = fill(1e-5, 3)) annotation(
          Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(pVsc.plug_n, qVsc.plug_p) annotation(
          Line(points = {{-20, 0}, {-14, 0}}, color = {0, 0, 255}));
        connect(transformer.plug1, plugIn) annotation(
          Line(points = {{90, 0}, {98, 0}, {98, 90}, {14, 90}, {14, 86}, {0, 86}, {0, 100}}, color = {0, 0, 255}));
        connect(pVsc.plug_p, gflBesC.plug) annotation(
          Line(points = {{-40, 0}, {-40, -0.2}, {-45.8, -0.2}}, color = {0, 0, 255}));
        connect(gflBesC.pin_ac, ground.p) annotation(
          Line(points = {{-46.2, -12}, {-38, -12}, {-38, -22}, {2, -22}, {2, -38}}, color = {0, 0, 255}));
        connect(ground.p, transformer.starpoint2) annotation(
          Line(points = {{2, -38}, {2, -22}, {60, -22}, {60, -20}}, color = {0, 0, 255}));
        connect(gflBesC.pRef, pqRef[1]) annotation(
          Line(points = {{-68, 0}, {-110, 0}, {-110, -65}}, color = {0, 0, 127}));
        connect(gflBesC.qRef, pqRef[2]) annotation(
          Line(points = {{-68, -12}, {-98, -12}, {-98, -55}, {-110, -55}}, color = {0, 0, 127}));
        connect(booleanStep1.y, gflBesC.u) annotation(
          Line(points = {{-43, 44}, {-48, 44}, {-48, 6}}, color = {255, 0, 255}));
        connect(qVsc.plug_n, idealCloser.plug_p) annotation(
          Line(points = {{6, 0}, {20, 0}}, color = {0, 0, 255}));
        connect(transformer.plug2, idealCloser.plug_n) annotation(
          Line(points = {{50, 0}, {40, 0}}, color = {0, 0, 255}));
        connect(booleanStep.y, idealCloser.control) annotation(
          Line(points = {{39, 70}, {30, 70}, {30, 12}}, color = {255, 0, 255}));
        annotation(
          experiment(Interval = 3e-05, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
          Documentation(info = "<html>
<p><span style=\"font-family: Segoe UI;\">MV-LV transformer + GFMBES without C filter</span></p>
</html>"),Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", origin = {124, 0}, rotation = 90), Line(points = {{-100, -48}, {100, 100}}, color = {0, 0, 0}), Line(points = {{16, -4}, {60, -4}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{24, -14}, {52, -26}}), Line(points = {{-100, -48}, {100, -48}}, color = {0, 0, 0}), Text(extent = {{-106, -68}, {0, -82}}, textColor = {0, 0, 0}, textString = "GFL PQ"), Ellipse(extent = {{-84, 66}, {-56, 38}}, lineColor = {0, 0, 0}), Line(points = {{-4, -94}, {-6, -46}}, color = {0, 0, 0}, pattern = LinePattern.None), Line(points = {{-4, 136}, {-6, 84}}, color = {0, 0, 0}, pattern = LinePattern.None), Rectangle(extent = {{516, 128}, {748, 52}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, lineThickness = 0.5), Ellipse(extent = {{-84, 82}, {-56, 54}}, lineColor = {0, 0, 0}), Line(points = {{-43, 50}, {14, 50}}, color = {0, 0, 0}), Line(points = {{-32.5, 42}, {3.5, 42}}, color = {0, 0, 0}), Line(points = {{-22, 34}, {-7, 34}}, color = {0, 0, 0}), Line(points = {{-14.5, 68}, {-14.5, 50}}, color = {0, 0, 0})}));
      end GflBesCTransfDy;

      model GfdBesCTransfDy "MV-LV transformer + GFDBES with C filter"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Modelica.Units.SI.Time tConn = -10 "Inverter connection time";
        parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Nominal apparent power";
        parameter Modelica.Units.SI.Voltage uDc = 750 "Nominal DC voltage";
        parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph line-to-line voltage (RMS), used for per-unit";
        parameter Real rfpu = lfpu/10 "Filter resistance (PU)";
        parameter Real lfpu = 0.05 "Filter inductance (PU)";
        parameter Real cfpu = 0.03 "Filter capacitance (PU)";
        parameter Real rcfpu = 1.6 "Filter resistance to C (PU)";
        final parameter Modelica.Units.SI.Resistance rf = rfpu*uAcNom*uAcNom/sNom "Filter resistance ";
        final parameter Modelica.Units.SI.Inductance lf = lfpu*uAcNom*uAcNom/(sNom*2*pi*f) "Filter inductance ";
        final parameter Modelica.Units.SI.Capacitance cf = cfpu*cNom "Filter capacitance";
        final parameter Modelica.Units.SI.Resistance rcf = rcfpu*zNom "Filter resistance to C";
        final parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom "Nominal Impedance";
        final parameter Modelica.Units.SI.Capacitance cNom = 1/(zNom*2*pi*f) "filter capacitance";
        parameter Modelica.Units.SI.Time delay = 0.0001 "Inverter time constant";
        parameter Real pGain = 0.05 "p.u. P-loop  control gain";
        parameter Real qGain = 0.05 "p.u. Q-loop  control gain";
        parameter Modelica.Units.SI.Time fo = 0.05 "first order time constant for P and Q calculation in VSC control";
        parameter Real secReg = 1 "secondary regulation, 1=ON 0=OFF";
        parameter Modelica.Units.SI.Voltage V1 = 20e3 "Transformer primary nominal line-to-line voltage (RMS)";
        parameter Modelica.Units.SI.Voltage V2 = 400 "Transformer secondary open circuit line-to-line voltage (RMS) @ primary nominal voltage";
        parameter Modelica.Units.SI.ApparentPower SNominal = 200e3 "Nominal apparent power transformer";
        parameter Real v_sc = 0.06 "Zcc pu, Impedance voltage drop pu transformer";
        parameter Real rcc = 0.0015 "Rcc pu transfomer, copper losses";
        parameter Real xcc = sqrt(v_sc^2 - rcc^2) "Xcc pu transfomer";
        final parameter Modelica.Units.SI.Power P_sc = rcc*SNominal "Short-circuit (copper) losses transformer";
        final parameter Modelica.Units.SI.Impedance ZbTr = V2*V2/SNominal "Zbase transformer";
        final parameter Modelica.Units.SI.Inductance lg = (xcc*ZbTr)/(2*pi*f) "L transformer";
        Modelica.Electrical.Polyphase.Sensors.AronSensor pVsc annotation(
          Placement(transformation(origin = {52, 50}, extent = {{-92, -60}, {-72, -40}})));
        Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qVsc annotation(
          Placement(transformation(origin = {48, 50}, extent = {{-62, -60}, {-42, -40}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug plugIn "from MV grid" annotation(
          Placement(transformation(extent = {{-10, 90}, {10, 110}}), iconTransformation(extent = {{-10, 90}, {10, 110}})));
        parameter Modelica.Electrical.Machines.Utilities.TransformerData transformerData(C1 = "D", C2 = "y", f = f, V1 = V1, V2 = V2, SNominal = SNominal, v_sc = v_sc, P_sc = P_sc) "Transformer data" annotation(
          Placement(transformation(extent = {{60, 30}, {80, 50}})));
        Modelica.Electrical.Machines.BasicMachines.Transformers.Dy.Dy01 transformer(n = transformerData.n, R1 = transformerData.R1, L1sigma = transformerData.L1sigma, R2 = transformerData.R2, L2sigma = transformerData.L2sigma, T1Ref = 293.15, alpha20_1(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T2Ref = 293.15, alpha20_2(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T1Operational = 293.15, T2Operational = 293.15) annotation(
          Placement(transformation(extent = {{90, -20}, {50, 20}})));
        Modelica.Blocks.Interfaces.RealInput pqRef[2] annotation(
          Placement(transformation(origin = {-110, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -76}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m = 3, Ron = fill(1e-5, 3), Goff = fill(1e-5, 3)) annotation(
          Placement(transformation(origin = {26, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep booleanStep[3](each startTime = tConn) annotation(
          Placement(transformation(extent = {{58, 64}, {38, 84}})));
        GfmDroopBesC gfdBesC(fNom = f, sNom = sNom, uDc = uDc, uAcNom = uAcNom, rf = rf, lf = lf, cf = cf, rcf = rcf, delay = delay, pGain = pGain, qGain = qGain, fo = fo, xcc = xcc, secReg = secReg) annotation(
          Placement(transformation(extent = {{-68, -16}, {-48, 4}})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(origin = {-4, -54}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(pVsc.plug_n, qVsc.plug_p) annotation(
          Line(points = {{-20, 0}, {-14, 0}}, color = {0, 0, 255}));
        connect(transformer.plug1, plugIn) annotation(
          Line(points = {{90, 0}, {90, 96}, {46, 96}, {46, 100}, {0, 100}}, color = {0, 0, 255}));
        connect(booleanStep.y, idealCloser.control) annotation(
          Line(points = {{37, 74}, {26, 74}, {26, 12}}, color = {255, 0, 255}));
        connect(transformer.plug2, idealCloser.plug_n) annotation(
          Line(points = {{50, 0}, {36, 0}}, color = {0, 0, 255}));
        connect(idealCloser.plug_p, qVsc.plug_n) annotation(
          Line(points = {{16, 0}, {6, 0}}, color = {0, 0, 255}));
        connect(pVsc.plug_p, gfdBesC.plug) annotation(
          Line(points = {{-40, 0}, {-47.8, -0.2}}, color = {0, 0, 255}));
        connect(gfdBesC.pin_ac, ground.p) annotation(
          Line(points = {{-48.2, -12}, {-38, -12}, {-38, -30}, {-4, -30}, {-4, -44}}, color = {0, 0, 255}));
        connect(transformer.starpoint2, ground.p) annotation(
          Line(points = {{60, -20}, {60, -30}, {-4, -30}, {-4, -44}}, color = {0, 0, 255}));
        connect(gfdBesC.pRef, pqRef[1]) annotation(
          Line(points = {{-70, 0}, {-110, 0}, {-110, -65}}, color = {0, 0, 127}));
        connect(gfdBesC.qRef, pqRef[2]) annotation(
          Line(points = {{-70, -12}, {-98, -12}, {-98, -55}, {-110, -55}}, color = {0, 0, 127}));
        annotation(
          experiment(Interval = 3e-05, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
          Documentation(info = "<html>
<p><span style=\"font-family: Segoe UI;\">MV-LV transformer + GFMBES without C filter</span></p>
</html>"),Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", origin = {124, 0}, rotation = 90), Line(points = {{-100, -48}, {100, 100}}, color = {0, 0, 0}), Line(points = {{16, -4}, {60, -4}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{24, -14}, {52, -26}}), Line(points = {{-100, -48}, {100, -48}}, color = {0, 0, 0}), Text(extent = {{-106, -68}, {0, -82}}, textColor = {0, 0, 0}, textString = "GFD PQ"), Ellipse(extent = {{-84, 66}, {-56, 38}}, lineColor = {0, 0, 0}), Line(points = {{-2, -94}, {-4, -46}}, color = {0, 0, 0}, pattern = LinePattern.None), Line(points = {{-2, 136}, {-4, 84}}, color = {0, 0, 0}, pattern = LinePattern.None), Rectangle(extent = {{210, 118}, {442, 42}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, lineThickness = 0.5), Ellipse(extent = {{-84, 82}, {-56, 54}}, lineColor = {0, 0, 0}), Line(points = {{-43, 50}, {14, 50}}, color = {0, 0, 0}), Line(points = {{-32.5, 42}, {3.5, 42}}, color = {0, 0, 0}), Line(points = {{-22, 34}, {-7, 34}}, color = {0, 0, 0}), Line(points = {{-14.5, 68}, {-14.5, 50}}, color = {0, 0, 0})}));
      end GfdBesCTransfDy;

      model LoadRLTransfDy "MV-LV transformer + RL load with known cos fi"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Modelica.Units.SI.Time tConn = -10 "Load insertion time";
        parameter Real cosfi = 0.9 "Power factor, cosfi";
        parameter Modelica.Units.SI.Voltage uAcNom = 400 "Nominal AC 3ph line-to-line voltage (RMS), used for per-unit";
        parameter Modelica.Units.SI.ActivePower pNom = 100e3 "Load active power";
        parameter Modelica.Units.SI.Voltage V1 = 20e3 "Transformer primary nominal line-to-line voltage (RMS)";
        parameter Modelica.Units.SI.Voltage V2 = 400 "Transformer secondary open circuit line-to-line voltage (RMS) @ primary nominal voltage";
        parameter Modelica.Units.SI.ApparentPower SNominal = 200e3 "Nominal apparent power transformer";
        parameter Real v_sc = 0.06 "Zcc (PU), Impedance voltage drop pu transformer";
        parameter Real rcc = 0.0015 "Rcc (PU) transfomer, copper losses";
        parameter Real xcc = sqrt(v_sc^2 - rcc^2) "Xcc (PU) transfomer";
        final parameter Modelica.Units.SI.Power P_sc = rcc*SNominal "Short-circuit (copper) losses transformer";
        final parameter Modelica.Units.SI.Impedance ZbTr = V2*V2/SNominal "Zbase transformer";
        final parameter Modelica.Units.SI.Inductance lg = (xcc*ZbTr)/(2*pi*f) "L transformer";
        Modelica.Electrical.Polyphase.Sensors.AronSensor pLoad annotation(
          Placement(transformation(origin = {64, 50}, extent = {{-92, -60}, {-72, -40}})));
        Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qLoad annotation(
          Placement(transformation(origin = {62, 50}, extent = {{-62, -60}, {-42, -40}})));
        Electric.LoadRL loadRL(cosfi = cosfi, f = f, uAcNom = uAcNom, pNom = pNom) annotation(
          Placement(transformation(extent = {{-64, -4}, {-44, 4}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug plugIn "from MV grid" annotation(
          Placement(transformation(extent = {{-10, 90}, {10, 110}}), iconTransformation(extent = {{-10, 90}, {10, 110}})));
        parameter Modelica.Electrical.Machines.Utilities.TransformerData transformerData(C1 = "D", C2 = "y", f = f, V1 = V1, V2 = V2, SNominal = SNominal, v_sc = v_sc, P_sc = P_sc) "Transformer data" annotation(
          Placement(transformation(extent = {{52, 30}, {72, 50}})));
        Modelica.Electrical.Machines.BasicMachines.Transformers.Dy.Dy01 transformer(n = transformerData.n, R1 = transformerData.R1, L1sigma = transformerData.L1sigma, R2 = transformerData.R2, L2sigma = transformerData.L2sigma, T1Ref = 293.15, alpha20_1(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T2Ref = 293.15, alpha20_2(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T1Operational = 293.15, T2Operational = 293.15) annotation(
          Placement(transformation(extent = {{90, -20}, {50, 20}})));
        Modelica.Electrical.Polyphase.Basic.Star star annotation(
          Placement(transformation(origin = {-82, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m = 3, Ron = fill(1e-5, 3), Goff = fill(1e-5, 3)) annotation(
          Placement(transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep booleanStep[3](each startTime = tConn) annotation(
          Placement(transformation(extent = {{62, 60}, {42, 80}})));
        Modelica.Electrical.Polyphase.Sensors.VoltageQuasiRMSSensor voltageRMSSensor annotation(
          Placement(transformation(origin = {-12, 4}, extent = {{-52, -52}, {-32, -32}})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(origin = {0, -76}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(pLoad.plug_n, qLoad.plug_p) annotation(
          Line(points = {{-8, 0}, {0, 0}}, color = {0, 0, 255}));
        connect(transformer.plug1, plugIn) annotation(
          Line(points = {{90, 0}, {98, 0}, {98, 86}, {0, 86}, {0, 100}}, color = {0, 0, 255}));
        connect(loadRL.positivePlug, star.plug_p) annotation(
          Line(points = {{-64, 0}, {-72, 0}}, color = {0, 0, 255}));
        connect(transformer.plug2, idealCloser.plug_n) annotation(
          Line(points = {{50, 0}, {46, 0}}, color = {0, 0, 255}));
        connect(idealCloser.plug_p, qLoad.plug_n) annotation(
          Line(points = {{26, 0}, {20, 0}}, color = {0, 0, 255}));
        connect(booleanStep.y, idealCloser.control) annotation(
          Line(points = {{41, 70}, {36, 70}, {36, 12}}, color = {255, 0, 255}));
        connect(voltageRMSSensor.plug_n, pLoad.plug_p) annotation(
          Line(points = {{-44, -38}, {-34, -38}, {-34, 0}, {-28, 0}}, color = {0, 0, 255}));
        connect(pLoad.plug_p, loadRL.negativePlug) annotation(
          Line(points = {{-28, 0}, {-44, 0}}, color = {0, 0, 255}));
        connect(voltageRMSSensor.plug_p, star.plug_p) annotation(
          Line(points = {{-64, -38}, {-68, -38}, {-68, 0}, {-72, 0}}, color = {0, 0, 255}));
        connect(transformer.starpoint2, ground.p) annotation(
          Line(points = {{60, -20}, {60, -60}, {0, -60}, {0, -66}}, color = {0, 0, 255}));
        connect(star.pin_n, ground.p) annotation(
          Line(points = {{-92, 0}, {-96, 0}, {-96, -60}, {0, -60}, {0, -66}}, color = {0, 0, 255}));
        annotation(
          experiment(Interval = 3e-05, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
          Documentation(info = "<html>
<p>RL load from cosfi and active power + transformer DY</p>
</html>"),Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", origin = {120, 0}, rotation = 90), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 24}, {12, -36}}, lineColor = {0, 0, 255}), Ellipse(extent = {{-12, 72}, {12, 50}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.None), Ellipse(extent = {{-12, 60}, {12, 38}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.None), Line(points = {{0, 72}, {0, 94}}, color = {0, 0, 255}), Line(points = {{0, 24}, {0, 38}}, color = {0, 0, 255}), Line(points = {{0, -54}, {0, -36}}, color = {0, 0, 255}), Text(extent = {{8, -4}, {76, -26}}, textColor = {0, 0, 255}, textString = "RL"), Line(points = {{-29, -76}, {28, -76}}, color = {0, 0, 0}), Line(points = {{-16.5, -84}, {19.5, -84}}, color = {0, 0, 0}), Line(points = {{-6, -92}, {9, -92}}, color = {0, 0, 0}), Line(points = {{0, -76}, {0, -66}}, color = {0, 0, 0}), Line(points = {{0, -66}, {20, -54}}, color = {0, 0, 0}), Line(points = {{0, -66}, {-20, -54}}, color = {0, 0, 0})}));
      end LoadRLTransfDy;

      model ImTransfDy "MV-LV transformer + Induction Motor 3ph squirrel cage"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Modelica.Units.SI.Voltage VNominal = 400/sqrt(3) "Nominal rms voltage per phase (scaling Im)";
        parameter Modelica.Units.SI.Time tStart1 = -10 "start time of Im connection";
        parameter Modelica.Units.SI.ApparentPower Sbase = 10e3 "Nominal Im apparent power";
        parameter Modelica.Units.SI.AngularVelocity wLoad = 1440.45*2*Modelica.Constants.pi/60 "Nominal Im load speed";
        parameter Modelica.Units.SI.AngularVelocity wInit = 1440.45*2*Modelica.Constants.pi/60 "Initial Im speed = Nominal load speed";
        parameter Modelica.Units.SI.Voltage V1 = 20e3 "Transformer primary nominal line-to-line voltage (RMS)";
        parameter Modelica.Units.SI.Voltage V2 = 400 "Transformer secondary open circuit line-to-line voltage (RMS) @ primary nominal voltage";
        parameter Modelica.Units.SI.ApparentPower SNominal = 200e3 "Nominal apparent power transformer";
        parameter Real v_sc = 0.06 "Zcc pu, Impedance voltage drop pu transformer";
        parameter Real rcc = 0.0015 "Rcc pu transfomer, copper losses";
        parameter Real xcc = sqrt(v_sc^2 - rcc^2) "Xcc pu transfomer";
        final parameter Modelica.Units.SI.Power P_sc = rcc*SNominal "Short-circuit (copper) losses transformer";
        final parameter Modelica.Units.SI.Impedance ZbTr = V2*V2/SNominal "Zbase transformer";
        final parameter Modelica.Units.SI.Inductance lg = (xcc*ZbTr)/(2*pi*f) "L transformer";
        Modelica.Electrical.Polyphase.Sensors.AronSensor pIm annotation(
          Placement(transformation(origin = {78, 50}, extent = {{-92, -60}, {-72, -40}})));
        Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qIm annotation(
          Placement(transformation(origin = {74, 50}, extent = {{-62, -60}, {-42, -40}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug plugIn "from MV grid" annotation(
          Placement(transformation(extent = {{-12, 90}, {8, 110}}), iconTransformation(extent = {{-12, 90}, {8, 110}})));
        parameter Modelica.Electrical.Machines.Utilities.TransformerData transformerData(C1 = "D", C2 = "y", f = f, V1 = V1, V2 = V2, SNominal = SNominal, v_sc = v_sc, P_sc = P_sc) "Transformer data" annotation(
          Placement(transformation(extent = {{52, 30}, {72, 50}})));
        Modelica.Electrical.Machines.BasicMachines.Transformers.Dy.Dy01 transformer(n = transformerData.n, R1 = transformerData.R1, L1sigma = transformerData.L1sigma, R2 = transformerData.R2, L2sigma = transformerData.L2sigma, T1Ref = 293.15, alpha20_1(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T2Ref = 293.15, alpha20_2(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T1Operational = 293.15, T2Operational = 293.15) annotation(
          Placement(transformation(extent = {{82, -20}, {42, 20}})));
        ImScScalable imScScalable(VNominal = VNominal, fNominal = f, tStart1 = tStart1, wLoad = wLoad, wInit = wInit, Sbase = Sbase) annotation(
          Placement(transformation(extent = {{-58, -16}, {-38, 4}})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(extent = {{42, -56}, {62, -36}})));
      equation
        connect(pIm.plug_n, qIm.plug_p) annotation(
          Line(points = {{6, 0}, {12, 0}}, color = {0, 0, 255}));
        connect(transformer.plug1, plugIn) annotation(
          Line(points = {{82, 0}, {90, 0}, {90, 86}, {-2, 86}, {-2, 100}}, color = {0, 0, 255}));
        connect(transformer.plug2, qIm.plug_n) annotation(
          Line(points = {{42, 0}, {32, 0}}, color = {0, 0, 255}));
        connect(pIm.plug_p, imScScalable.plug) annotation(
          Line(points = {{-14, 0}, {-38, 0}}, color = {0, 0, 255}));
        connect(ground.p, transformer.starpoint2) annotation(
          Line(points = {{52, -36}, {52, -20}}, color = {0, 0, 255}));
        annotation(
          experiment(Interval = 3e-05, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
          Documentation(info = "<html>
<p>induction motor + transformer DY</p>
</html>"),Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", origin = {120, 0}, rotation = 90), Polygon(points = {{-14, -28}, {-4, 10}, {18, -2}, {-14, -28}}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-38, -26}, {-2, -36}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.None), Ellipse(extent = {{-96, 0}, {-32, -62}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-88, -14}, {-40, -48}}, textColor = {0, 0, 0}, textString = "M"), Line(points = {{8, 4}, {16, 14}, {34, 22}, {56, 22}, {80, 12}}, color = {0, 0, 0}), Ellipse(extent = {{-78, 72}, {-52, 48}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.None), Ellipse(extent = {{-78, 60}, {-52, 36}}, lineColor = {0, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.None), Line(points = {{-64, 0}, {-64, 38}}, color = {0, 0, 255}), Line(points = {{-64, 72}, {-64, 88}, {-2, 88}, {-2, 96}}, color = {0, 0, 255}), Line(points = {{48, 46}, {63, 46}}, color = {0, 0, 0}), Line(points = {{37.5, 54}, {73.5, 54}}, color = {0, 0, 0}), Line(points = {{27, 62}, {84, 62}}, color = {0, 0, 0}), Line(points = {{55.5, 80}, {55.5, 62}}, color = {0, 0, 0})}));
      end ImTransfDy;

      model LineRLCMV "MT da Cagliari, conduttore aereo 150mmq con C verso terra"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Distance len = 50 "cable lenght";
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Real R1(final unit = "Ohm/km") = 0.23 "Resistance per unit length";
        parameter Real X1(final unit = "Ohm/km") = 0.34 "Reactance per unit length";
        parameter Real C1(final unit = "F/km") = 0.01e-6 "Capacitance per unit length";
        Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = R1*(len/1000)*ones(3)) annotation(
          Placement(transformation(extent = {{-78, 14}, {-58, 34}})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = X1*(len/1000)/(2*pi*f)*ones(3)) annotation(
          Placement(transformation(extent = {{-46, 14}, {-26, 34}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug positivePlug annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.NegativePlug negativePlug annotation(
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
        Modelica.Electrical.Polyphase.Basic.Capacitor capacitor(C = C1*(len/1000)*ones(3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {26, 10})));
        Modelica.Electrical.Polyphase.Basic.Star star annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {26, -16})));
        Modelica.Electrical.Analog.Interfaces.NegativePin gnd annotation(
          Placement(transformation(extent = {{-10, -50}, {10, -30}})));
      equation
        connect(resistor.plug_n, inductor.plug_p) annotation(
          Line(points = {{-58, 24}, {-46, 24}}, color = {0, 0, 255}));
        connect(inductor.plug_n, negativePlug) annotation(
          Line(points = {{-26, 24}, {86, 24}, {86, 0}, {100, 0}}, color = {0, 0, 255}));
        connect(resistor.plug_p, positivePlug) annotation(
          Line(points = {{-78, 24}, {-86, 24}, {-86, 0}, {-100, 0}}, color = {0, 0, 255}));
        connect(inductor.plug_n, capacitor.plug_p) annotation(
          Line(points = {{-26, 24}, {26, 24}, {26, 20}}, color = {0, 0, 255}));
        connect(capacitor.plug_n, star.plug_p) annotation(
          Line(points = {{26, 0}, {26, -6}}, color = {0, 0, 255}));
        connect(star.pin_n, gnd) annotation(
          Line(points = {{26, -26}, {26, -30}, {12, -30}, {12, -26}, {0, -26}, {0, -40}}, color = {0, 0, 255}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-102, 88}, {98, 48}}, textColor = {0, 0, 255}, textString = "%name")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})));
      end LineRLCMV;

      model LineRLMV "MT da Cagliari, conduttore aereo 150mmq (manca 0.01microF/km)"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Distance len = 50 "lenght";
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Real R1(final unit = "Ohm/km") = 0.23 "Resistance per unit length";
        parameter Real X1(final unit = "Ohm/km") = 0.34 "Reactance per unit length";
        Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = R1*(len/1000)*ones(3)) annotation(
          Placement(transformation(extent = {{-36, -10}, {-16, 10}})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = X1*(len/1000)/(2*pi*f)*ones(3)) annotation(
          Placement(transformation(extent = {{22, -10}, {42, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug positivePlug annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.NegativePlug negativePlug annotation(
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
      equation
        connect(resistor.plug_n, inductor.plug_p) annotation(
          Line(points = {{-16, 0}, {22, 0}}, color = {0, 0, 255}));
        connect(inductor.plug_n, negativePlug) annotation(
          Line(points = {{42, 0}, {100, 0}}, color = {0, 0, 255}));
        connect(resistor.plug_p, positivePlug) annotation(
          Line(points = {{-36, 0}, {-100, 0}}, color = {0, 0, 255}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-102, 88}, {98, 48}}, textColor = {0, 0, 255}, textString = "%name")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})));
      end LineRLMV;

      model CableRLMV "cable 150mmq"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Distance len = 200 "lenght";
        parameter Modelica.Units.SI.Frequency f = 50 "Nominal frequency";
        parameter Real R1(final unit = "Ohm/km") = 0.206 "Resistance per unit length";
        parameter Real X1(final unit = "Ohm/km") = 0.117 "Reactance per unit length";
        Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = R1*(len/1000)*ones(3)) annotation(
          Placement(transformation(extent = {{-36, -10}, {-16, 10}})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = X1*(len/1000)/(2*pi*f)*ones(3)) annotation(
          Placement(transformation(extent = {{22, -10}, {42, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug positivePlug annotation(
          Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.NegativePlug negativePlug annotation(
          Placement(transformation(extent = {{90, -10}, {110, 10}})));
      equation
        connect(resistor.plug_n, inductor.plug_p) annotation(
          Line(points = {{-16, 0}, {22, 0}}, color = {0, 0, 255}));
        connect(inductor.plug_n, negativePlug) annotation(
          Line(points = {{42, 0}, {100, 0}}, color = {0, 0, 255}));
        connect(resistor.plug_p, positivePlug) annotation(
          Line(points = {{-36, 0}, {-100, 0}}, color = {0, 0, 255}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}}), graphics = {Rectangle(extent = {{-100, 40}, {100, -40}}, lineColor = {28, 108, 200}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-100, 80}, {100, 40}}, textColor = {0, 0, 255}, textString = "%name"), Text(extent = {{-74, 20}, {74, -22}}, textColor = {28, 108, 200}, textString = "R-X")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -40}, {100, 40}})));
      end CableRLMV;

      model VsmBesCTransfDy "MV-LV transformer + VSMBES with C filter"
        import Modelica.Constants.pi;
        parameter Modelica.Units.SI.Frequency f = 50 "Frequenza nominale";
        parameter Modelica.Units.SI.Time tConn = -10 "Inverter connection time";
        parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Potenza nominale";
        parameter Modelica.Units.SI.Voltage uDc = 750 "Val. costante tensione DC";
        parameter Modelica.Units.SI.Voltage uAcNom = 400 "Val. nominale tensione AC trifase rms (per PU)";
        parameter Real rfpu = lfpu/10 "filter resistance (PU)";
        parameter Real lfpu = 0.05 "filter inductance (PU)";
        parameter Real cfpu = 0.03 "filter capacitance (PU)";
        parameter Real rcfpu = 1.6 "filter resistanceC (PU)";
        final parameter Modelica.Units.SI.Resistance rf = rfpu*zNom "filter resistance ";
        final parameter Modelica.Units.SI.Resistance rcf = rcfpu*zNom "filter resistanceC ";
        final parameter Modelica.Units.SI.Inductance lf = lfpu*lNom "filter inductance ";
        final parameter Modelica.Units.SI.Capacitance cf = cfpu*cNom "filter capacitance";
        parameter Modelica.Units.SI.Time delay = 0.0001 "Inverter time constant";
        final parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom;
        parameter Modelica.Units.SI.Reactance x = 2*pi*f*lf/zNom "Reattanza Thevenin VSC (PU)";
        final parameter Modelica.Units.SI.Inductance lNom = zNom/(2*pi*f) "induttanza base (PU)";
        final parameter Modelica.Units.SI.Capacitance cNom = 1/(zNom*2*pi*f) "filter capacitance";
        parameter Modelica.Units.SI.Inductance lfg = 0 "filter LCL grid side inductance";
        //  parameter Modelica.Units.SI.Inductance lg = 2.44e-5 "grid connection inductance (transformer)";
        parameter Real rv = 0.02 "resistenza virtuale (PU)";
        parameter Real lv = 0.1 "induttanza virtuale (PU)";
        parameter Modelica.Units.SI.Time h = 4 "costante inerzia";
        parameter Real csiMec = 0.7 "smorzamento meccanico swing equation";
        parameter Modelica.Units.SI.Time te = 1 "costante tempo eccitazione";
        parameter Real pGain = 0.05 "P-loop  control gain";
        parameter Real qGain = 0.05 "Q-loop  control gain";
        parameter Modelica.Units.SI.Time fo = 0.02 "first order time constant for P and Q droop";
        parameter Modelica.Units.SI.Voltage V1 = 20e3 "Transformer primary nominal line-to-line voltage (RMS)";
        parameter Modelica.Units.SI.Voltage V2 = 400 "Transformer secondary open circuit line-to-line voltage (RMS) @ primary nominal voltage";
        parameter Modelica.Units.SI.ApparentPower SNominal = 200e3 "Nominal apparent power transformer";
        parameter Real v_sc = 0.06 "Zcc pu, Impedance voltage drop pu transformer";
        parameter Real rcc = 0.0015 "Rcc pu transfomer, copper losses";
        parameter Real xcc = sqrt(v_sc^2 - rcc^2) "Xcc pu transfomer";
        final parameter Modelica.Units.SI.Power P_sc = rcc*SNominal "Short-circuit (copper) losses transformer";
        final parameter Modelica.Units.SI.Impedance ZbTr = V2*V2/SNominal "Zbase transformer";
        final parameter Modelica.Units.SI.Inductance lg = (xcc*ZbTr)/(2*pi*f) "L transformer";
        Modelica.Electrical.Polyphase.Sensors.AronSensor pVsc annotation(
          Placement(transformation(origin = {52, 50}, extent = {{-92, -60}, {-72, -40}})));
        Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qVsc annotation(
          Placement(transformation(origin = {48, 50}, extent = {{-62, -60}, {-42, -40}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug plugIn "from MV grid" annotation(
          Placement(transformation(extent = {{-10, 88}, {10, 108}}), iconTransformation(extent = {{-10, 88}, {10, 108}})));
        parameter Modelica.Electrical.Machines.Utilities.TransformerData transformerData(C1 = "D", C2 = "y", f = f, V1 = V1, V2 = V2, SNominal = SNominal, v_sc = v_sc, P_sc = P_sc) "Transformer data" annotation(
          Placement(transformation(extent = {{60, 30}, {80, 50}})));
        Modelica.Electrical.Machines.BasicMachines.Transformers.Dy.Dy01 transformer(n = transformerData.n, R1 = transformerData.R1, L1sigma = transformerData.L1sigma, R2 = transformerData.R2, L2sigma = transformerData.L2sigma, T1Ref = 293.15, alpha20_1(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T2Ref = 293.15, alpha20_2(displayUnit = "1/K") = Modelica.Electrical.Machines.Thermal.Constants.alpha20Zero, T1Operational = 293.15, T2Operational = 293.15) annotation(
          Placement(transformation(extent = {{90, -20}, {50, 20}})));
        Modelica.Blocks.Interfaces.RealInput pqRef[2] annotation(
          Placement(transformation(origin = {-110, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -76}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Electrical.Polyphase.Ideal.IdealClosingSwitch idealCloser(final m = 3, Ron = fill(1e-5, 3), Goff = fill(1e-5, 3)) annotation(
          Placement(transformation(origin = {26, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.BooleanStep booleanStep[3](each startTime = tConn) annotation(
          Placement(transformation(extent = {{58, 64}, {38, 84}})));
        GfmVsmBesC vsmBesC(fNom = f, sNom = sNom, uDc = uDc, uAcNom = uAcNom, rfpu = rfpu, lfpu = lfpu, cfpu = cfpu, rcfpu = rcfpu, delay = delay, x = x, lfg = lfg, lg = lg, rv = rv, lv = lv, h = h, csiMec = csiMec, te = te, pGain = pGain, qGain = qGain, fo = fo) annotation(
          Placement(transformation(origin = {-54, -6}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Interfaces.BooleanInput mode "true=island" annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {-60, -108}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = 180, origin = {120, -78})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(origin = {0, -52}, extent = {{-10, -10}, {10, 10}})));
      equation
        connect(pVsc.plug_n, qVsc.plug_p) annotation(
          Line(points = {{-20, 0}, {-14, 0}}, color = {0, 0, 255}));
        connect(transformer.plug1, plugIn) annotation(
          Line(points = {{90, 0}, {98, 0}, {98, 88}, {14, 88}, {14, 84}, {0, 84}, {0, 98}}, color = {0, 0, 255}));
        connect(booleanStep.y, idealCloser.control) annotation(
          Line(points = {{37, 74}, {26, 74}, {26, 12}}, color = {255, 0, 255}));
        connect(transformer.plug2, idealCloser.plug_n) annotation(
          Line(points = {{50, 0}, {36, 0}}, color = {0, 0, 255}));
        connect(idealCloser.plug_p, qVsc.plug_n) annotation(
          Line(points = {{16, 0}, {6, 0}}, color = {0, 0, 255}));
        connect(pVsc.plug_p, vsmBesC.plug) annotation(
          Line(points = {{-40, 0}, {-42, 0}, {-42, -0.2}, {-43.8, -0.2}}, color = {0, 0, 255}));
        connect(mode, vsmBesC.mode) annotation(
          Line(points = {{-60, -108}, {-60, -16}}, color = {255, 0, 255}));
        connect(vsmBesC.pRef, pqRef[1]) annotation(
          Line(points = {{-66, 0}, {-110, 0}, {-110, -65}}, color = {0, 0, 127}));
        connect(vsmBesC.qRef, pqRef[2]) annotation(
          Line(points = {{-66, -12}, {-102, -12}, {-102, -60}, {-110, -60}, {-110, -55}}, color = {0, 0, 127}));
        connect(vsmBesC.pin_ac, ground.p) annotation(
          Line(points = {{-44.2, -12}, {-42, -12}, {-42, -10}, {-38, -10}, {-38, -24}, {0, -24}, {0, -42}}, color = {0, 0, 255}));
        connect(transformer.starpoint2, ground.p) annotation(
          Line(points = {{60, -20}, {54, -20}, {54, -24}, {0, -24}, {0, -42}}, color = {0, 0, 255}));
        annotation(
          experiment(Interval = 3e-05, Tolerance = 1e-06, __Dymola_Algorithm = "Dassl"),
          Documentation(info = "<html>
      <p><span style=\"font-family: Segoe UI;\">MV-LV transformer + GFMBES without C filter</span></p>
      </html>"),
          Icon(graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 20}, {100, -20}}, textString = "%name", origin = {130, 46}, rotation = 90), Line(points = {{-100, -48}, {100, 100}}, color = {0, 0, 0}), Line(points = {{8, -4}, {66, -4}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{24, -14}, {52, -26}}), Line(points = {{-100, -48}, {100, -48}}, color = {0, 0, 0}), Text(extent = {{-104, -68}, {2, -82}}, textColor = {0, 0, 0}, textString = "VSM PQ"), Ellipse(extent = {{-84, 66}, {-56, 38}}, lineColor = {0, 0, 0}), Line(points = {{-2, -94}, {-4, -46}}, color = {0, 0, 0}, pattern = LinePattern.None), Line(points = {{-2, 136}, {-4, 84}}, color = {0, 0, 0}, pattern = LinePattern.None), Text(extent = {{20, -66}, {102, -82}}, textColor = {0, 0, 0}, textString = "Island"), Line(points = {{-80, 100}, {-78, 54}, {-82, 58}, {128, 58}, {132, 34}}, color = {0, 0, 0}, pattern = LinePattern.None, thickness = 0.5), Rectangle(extent = {{-90, 94}, {142, 18}}, lineColor = {0, 0, 0}, pattern = LinePattern.None, lineThickness = 0.5), Ellipse(extent = {{-84, 82}, {-56, 54}}, lineColor = {0, 0, 0}), Line(points = {{-43, 50}, {14, 50}}, color = {0, 0, 0}), Line(points = {{-32.5, 42}, {3.5, 42}}, color = {0, 0, 0}), Line(points = {{-22, 34}, {-7, 34}}, color = {0, 0, 0}), Line(points = {{-14.5, 68}, {-14.5, 50}}, color = {0, 0, 0})}));
      end VsmBesCTransfDy;
    end MV;

    model GfmVsmBesC
      import Modelica.Constants.pi;
      parameter Modelica.Units.SI.Frequency fNom = 50 "Frequenza nominale";
      parameter Modelica.Units.SI.ApparentPower sNom = 100e3 "Potenza nominale";
      parameter Modelica.Units.SI.Voltage uDc = 750 "Val. nominale tensione DC (per PU)";
      parameter Modelica.Units.SI.Voltage uAcNom = 400 "Val. nominale tensione AC trifase rms (per PU)";
      parameter Real rfpu = lfpu/10 "filter resistance (PU)";
      parameter Real lfpu = 0.059 "filter inductance (PU)";
      parameter Real cfpu = 0.007 "filter capacitance (PU)";
      parameter Real rcfpu = 1.6 "filter resistanceC (PU)";
      final parameter Modelica.Units.SI.Resistance rf = rfpu*zNom "filter resistance ";
      final parameter Modelica.Units.SI.Resistance rcf = rcfpu*zNom "filter resistance ";
      final parameter Modelica.Units.SI.Inductance lf = lfpu*lNom "filter inductance ";
      final parameter Modelica.Units.SI.Capacitance cf = cfpu*cNom "6e-6 filter capacitance";
      parameter Modelica.Units.SI.Time delay = 0.0001 "Inverter time constant";
      final parameter Modelica.Units.SI.Impedance zNom = uAcNom^2/sNom;
      parameter Modelica.Units.SI.Reactance x = 2*pi*fNom*lf/zNom "Reattanza Thevenin VSC (PU)";
      final parameter Modelica.Units.SI.Inductance lNom = zNom/(2*pi*fNom) "induttanza base (PU)";
      final parameter Modelica.Units.SI.Capacitance cNom = 1/(zNom*2*pi*fNom) "filter capacitance";
      parameter Modelica.Units.SI.Inductance lfg = 0 "filter LCL grid side inductance";
      parameter Modelica.Units.SI.Inductance lg = 2.44e-5 "grid connection inductance (transformer)";
      parameter Real rv = 0.02 "resistenza virtuale (PU)";
      parameter Real lv = 0.1 "induttanza virtuale (PU)";
      parameter Modelica.Units.SI.Time h = 4 "costante inerzia";
      parameter Real csiMec = 0.7 "smorzamento meccanico swing equation";
      parameter Modelica.Units.SI.Time te = 1 "costante tempo eccitazione";
      parameter Real pGain = 0.05 "P-loop  control gain";
      parameter Real qGain = 0.05 "Q-loop  control gain";
      parameter Modelica.Units.SI.Time fo = 0.02 "first order time constant for P and Q droop";
      Modelica.Electrical.Analog.Interfaces.NegativePin pin_ac annotation(
        Placement(transformation(origin = {100, -62}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {98, -60}, extent = {{-10, -10}, {10, 10}})));
      Inv inv(delay = delay) annotation(
        Placement(transformation(extent = {{-36, 34}, {-16, 54}})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage uDcFem(V = uDc) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-78, 46})));
      Modelica.Electrical.Polyphase.Sensors.PotentialSensor vscVolt(phi(each fixed = false)) annotation(
        Placement(transformation(extent = {{8, -8}, {-8, 8}}, rotation = 270, origin = {74, 70})));
      Modelica.Electrical.Analog.Sensors.PowerSensor pDC annotation(
        Placement(transformation(extent = {{-66, 42}, {-46, 62}})));
      Modelica.Electrical.Polyphase.Sensors.CurrentSensor vscCurr annotation(
        Placement(transformation(origin = {6, 50}, extent = {{-8, 8}, {8, -8}})));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug annotation(
        Placement(transformation(origin = {100, 50}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {102, 58}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = rf*ones(3)) annotation(
        Placement(transformation(extent = {{24, 40}, {44, 60}})));
      Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = lf*ones(3)) annotation(
        Placement(transformation(extent = {{50, 40}, {70, 60}})));
      Modelica.Blocks.Interfaces.RealInput pRef annotation(
        Placement(transformation(origin = {-140, -6}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput qRef annotation(
        Placement(transformation(origin = {-139, -47}, extent = {{-19, -19}, {19, 19}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Blocks.Park.DqToAbc dqToAbc annotation(
        Placement(transformation(extent = {{6, -48}, {26, -28}})));
      Modelica.Electrical.Polyphase.Basic.Capacitor capacitor(C = cf*ones(3), v(start = 1*ones(3), each fixed = false)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {74, -4})));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {74, -36})));
      Modelica.Blocks.Sources.Constant const(k = 0) annotation(
        Placement(transformation(extent = {{-118, -92}, {-98, -72}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistorC(R = rcf*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {74, 26})));
      Blocks.Vsm.GfmVsmControl VSMcontrol(fNom = fNom, sNom = sNom, uDc = uDc, uAcNom = uAcNom, lf = lf, lfg = lfg, lg = lg, rv = rv, lv = lv, h = h, csiMec = csiMec, te = te, pGain = pGain, qGain = qGain, fo = fo) annotation(
        Placement(transformation(origin = {-46, -44}, extent = {{-28, -28}, {28, 28}})));
      Modelica.Blocks.Interfaces.BooleanInput mode "true=island" annotation(
        Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {-46, -108}), iconTransformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {-60, -100})));
    equation
      connect(uDcFem.n, inv.pin_n) annotation(
        Line(points = {{-78, 36}, {-78, 30}, {-56, 30}, {-56, 37.8}, {-36, 37.8}}, color = {0, 0, 255}));
      connect(pDC.nv, uDcFem.n) annotation(
        Line(points = {{-56, 42}, {-56, 30}, {-78, 30}, {-78, 36}}, color = {0, 0, 255}));
      connect(pDC.pv, pDC.nc) annotation(
        Line(points = {{-56, 62}, {-46, 62}, {-46, 52}}, color = {0, 0, 255}));
      connect(pDC.pc, uDcFem.p) annotation(
        Line(points = {{-66, 52}, {-66, 56}, {-78, 56}}, color = {0, 0, 255}));
      connect(pDC.nc, inv.pin_p) annotation(
        Line(points = {{-46, 52}, {-42, 52}, {-42, 50}, {-36, 50}}, color = {0, 0, 255}));
      connect(inv.pin_ac, pin_ac) annotation(
        Line(points = {{-16.2, 38}, {-8, 38}, {-8, 30}, {42, 30}, {42, -62}, {100, -62}}, color = {0, 0, 255}));
      connect(resistor.plug_n, inductor.plug_p) annotation(
        Line(points = {{44, 50}, {50, 50}}, color = {0, 0, 255}));
      connect(vscVolt.plug_p, inductor.plug_n) annotation(
        Line(points = {{74, 62}, {74, 50}, {70, 50}}, color = {0, 0, 255}));
      connect(inv.pin_n, pin_ac) annotation(
        Line(points = {{-36, 37.8}, {-46, 37.8}, {-46, 24}, {-8, 24}, {-8, 30}, {42, 30}, {42, -62}, {100, -62}}, color = {0, 0, 255}));
      connect(vscCurr.plug_n, resistor.plug_p) annotation(
        Line(points = {{14, 50}, {24, 50}}, color = {0, 0, 255}));
      connect(inv.plug, vscCurr.plug_p) annotation(
        Line(points = {{-16, 50}, {-2, 50}}, color = {0, 0, 255}));
      connect(dqToAbc.abc, inv.u) annotation(
        Line(points = {{26.2, -38}, {34, -38}, {34, 20}, {-26, 20}, {-26, 32.8}}, color = {0, 0, 127}));
      connect(plug, inductor.plug_n) annotation(
        Line(points = {{100, 50}, {70, 50}}, color = {0, 0, 255}));
      connect(star.pin_n, pin_ac) annotation(
        Line(points = {{74, -46}, {74, -62}, {100, -62}}, color = {0, 0, 255}));
      connect(star.plug_p, capacitor.plug_n) annotation(
        Line(points = {{74, -26}, {74, -14}}, color = {0, 0, 255}));
      connect(capacitor.plug_p, resistorC.plug_n) annotation(
        Line(points = {{74, 6}, {74, 16}}, color = {0, 0, 255}));
      connect(resistorC.plug_p, inductor.plug_n) annotation(
        Line(points = {{74, 36}, {74, 50}, {70, 50}}, color = {0, 0, 255}));
      connect(dqToAbc.theta, VSMcontrol.theta) annotation(
        Line(points = {{6.8, -43.6}, {-15.2, -43.6}, {-15.2, -58}}, color = {0, 0, 127}));
      connect(VSMcontrol.pRef, pRef) annotation(
        Line(points = {{-79.6, -38.4}, {-112, -38.4}, {-112, -6}, {-140, -6}}, color = {0, 0, 127}));
      connect(VSMcontrol.qRef, qRef) annotation(
        Line(points = {{-79.6, -49.6}, {-139, -49.6}, {-139, -47}}, color = {0, 0, 127}));
      connect(const.y, VSMcontrol.pvRef) annotation(
        Line(points = {{-97, -82}, {-90, -82}, {-90, -60.8}, {-79.6, -60.8}}, color = {0, 0, 127}));
      connect(VSMcontrol.qvRef, const.y) annotation(
        Line(points = {{-79.6, -72}, {-90, -72}, {-90, -82}, {-97, -82}}, color = {0, 0, 127}));
      connect(vscVolt.phi, VSMcontrol.vabc) annotation(
        Line(points = {{74, 78.8}, {74, 82}, {-102, 82}, {-102, -16}, {-79.6, -16}}, color = {0, 0, 127}));
      connect(vscCurr.i, VSMcontrol.iabc) annotation(
        Line(points = {{6, 58.8}, {6, 72}, {-94, 72}, {-94, -27.2}, {-79.6, -27.2}}, color = {0, 0, 127}));
      connect(VSMcontrol.mdqV, dqToAbc.dq) annotation(
        Line(points = {{-15.2, -27.2}, {-4, -27.2}, {-4, -34.8}, {6.8, -34.8}}, color = {0, 0, 127}));
      connect(mode, VSMcontrol.mode) annotation(
        Line(points = {{-46, -108}, {-46, -72}}, color = {255, 0, 255}));
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = false), graphics = {Text(textColor = {0, 0, 255}, extent = {{-106, 146}, {104, 104}}, textString = "%name"), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, 0}, extent = {{-96, 70}, {-30, 52}}, textString = "pRef"), Text(origin = {2, 0}, extent = {{-96, -50}, {-30, -68}}, textString = "qRef"), Line(points = {{-86, 14}, {-30, 14}}), Rectangle(extent = {{-24, 84}, {74, -80}}, lineColor = {28, 108, 200}), Line(points = {{-24, -80}, {74, 84}}, color = {28, 108, 200}), Line(points = {{10, -54}, {20, -36}, {30, -36}, {44, -68}, {54, -68}, {62, -50}}), Line(points = {{92, 58}, {74, 58}}, color = {28, 108, 200}), Line(points = {{90, -60}, {74, -60}}, color = {28, 108, 200}), Line(points = {{-24, 42}, {-58, 42}, {-58, 14}}, color = {28, 108, 200}), Line(points = {{-24, -38}, {-58, -38}, {-58, -10}}, color = {28, 108, 200}), Line(points = {{-8, 60}, {34, 60}}), Rectangle(fillPattern = FillPattern.Solid, extent = {{-72, 0}, {-44, -12}}), Text(extent = {{-34, 30}, {84, -28}}, textColor = {0, 0, 0}, textString = "VSM")}),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-120, -100}, {100, 100}})),
        Documentation(info = "<html>
    <p>Models a, AC-interfaced Battery energy storage system.</p>
    <p>It contains a Constant DC source, and a Voltage Sourced Converter (VSC), and its control</p>
    <p>The VSC is simulated generating its fundamental-frequency component three-phase balanced set of electromotive forces, which feeds an R-L-C filter.</p>
    <p>The control system is VSM ... </p>
    <p>The internal structure of this component is organised in a way that favours new models to be easily built, e.g. with a richer DC-side network or control logic </p>
    </html>"));
    end GfmVsmBesC;

    package TestsBreakerCloser
      model TestArcQuench
        BreakerArc breaker annotation(
          Placement(transformation(extent = {{-84, 20}, {-72, 32}})));
        Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(m = 3, V = fill(100, 3), f = fill(50, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-118, 10})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = fill(1e-3, 3)) annotation(
          Placement(transformation(extent = {{-56, 16}, {-36, 36}})));
        Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(10, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-18, 10})));
        Modelica.Electrical.Polyphase.Basic.Star star annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-66, -22})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(extent = {{-76, -58}, {-56, -38}})));
        Modelica.Blocks.Sources.BooleanStep open(startTime = 0.1) annotation(
          Placement(transformation(extent = {{-122, 32}, {-102, 52}})));
        Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage1(m = 3, V = fill(100, 3), f = fill(50, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {12, 10})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor1(L = fill(1e-3, 3)) annotation(
          Placement(transformation(extent = {{74, 16}, {94, 36}})));
        Modelica.Electrical.Polyphase.Basic.Resistor resistor1(R = fill(10, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {112, 10})));
        Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {64, -22})));
        Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
          Placement(transformation(extent = {{54, -58}, {74, -38}})));
        Modelica.Blocks.Sources.BooleanStep open1[3](startTime = 0.1*ones(3)) annotation(
          Placement(transformation(extent = {{8, 32}, {28, 52}})));
        Modelica.Electrical.Polyphase.Ideal.OpenerWithArc switch(Ron = fill(1e-5, 3), Goff = fill(1e-5, 3), V0 = fill(30, 3), dVdt = fill(10000, 3), Vmax = fill(60, 3)) annotation(
          Placement(transformation(extent = {{38, 16}, {58, 36}})));
      equation
        connect(breaker.pPlug, sineVoltage.plug_p) annotation(
          Line(points = {{-84, 26}, {-118, 26}, {-118, 20}}, color = {0, 0, 255}));
        connect(inductor.plug_n, resistor.plug_p) annotation(
          Line(points = {{-36, 26}, {-18, 26}, {-18, 20}}, color = {0, 0, 255}));
        connect(inductor.plug_p, breaker.nPlug) annotation(
          Line(points = {{-56, 26}, {-72, 26}}, color = {0, 0, 255}));
        connect(sineVoltage.plug_n, resistor.plug_n) annotation(
          Line(points = {{-118, 0}, {-118, -4}, {-18, -4}, {-18, 0}}, color = {0, 0, 255}));
        connect(star.plug_p, resistor.plug_n) annotation(
          Line(points = {{-66, -12}, {-66, -4}, {-18, -4}, {-18, 0}}, color = {0, 0, 255}));
        connect(star.pin_n, ground.p) annotation(
          Line(points = {{-66, -32}, {-66, -38}}, color = {0, 0, 255}));
        connect(open.y, breaker.u) annotation(
          Line(points = {{-101, 42}, {-80, 42}, {-80, 33}}, color = {255, 0, 255}));
        connect(inductor1.plug_n, resistor1.plug_p) annotation(
          Line(points = {{94, 26}, {112, 26}, {112, 20}}, color = {0, 0, 255}));
        connect(sineVoltage1.plug_n, resistor1.plug_n) annotation(
          Line(points = {{12, 0}, {12, -4}, {112, -4}, {112, 0}}, color = {0, 0, 255}));
        connect(star1.plug_p, resistor1.plug_n) annotation(
          Line(points = {{64, -12}, {64, -4}, {112, -4}, {112, 0}}, color = {0, 0, 255}));
        connect(star1.pin_n, ground1.p) annotation(
          Line(points = {{64, -32}, {64, -38}}, color = {0, 0, 255}));
        connect(inductor1.plug_p, switch.plug_n) annotation(
          Line(points = {{74, 26}, {58, 26}}, color = {0, 0, 255}));
        connect(switch.plug_p, sineVoltage1.plug_p) annotation(
          Line(points = {{38, 26}, {12, 26}, {12, 20}}, color = {0, 0, 255}));
        connect(open1.y, switch.control) annotation(
          Line(points = {{29, 42}, {40, 42}, {40, 46}, {48, 46}, {48, 38}}, color = {255, 0, 255}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -60}, {80, 60}})),
          experiment(StopTime = 0.2, __Dymola_Algorithm = "Dassl"));
      end TestArcQuench;

      model TestArcQuenchClose
        Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage(m = 3, V = fill(100, 3), f = fill(50, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-114, 8})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor(L = fill(1e-3, 3)) annotation(
          Placement(transformation(extent = {{-52, 14}, {-32, 34}})));
        Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(10, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-14, 8})));
        Modelica.Electrical.Polyphase.Basic.Star star annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-62, -24})));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(transformation(extent = {{-72, -60}, {-52, -40}})));
        Modelica.Blocks.Sources.BooleanStep close(startTime = 0.1) annotation(
          Placement(transformation(extent = {{-118, 32}, {-98, 52}})));
        CloserArc closerArc annotation(
          Placement(transformation(extent = {{-84, 18}, {-72, 30}})));
        Modelica.Electrical.Polyphase.Sources.SineVoltage sineVoltage1(m = 3, V = fill(100, 3), f = fill(50, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {14, 10})));
        Modelica.Electrical.Polyphase.Basic.Inductor inductor1(L = fill(1e-3, 3)) annotation(
          Placement(transformation(extent = {{76, 16}, {96, 36}})));
        Modelica.Electrical.Polyphase.Basic.Resistor resistor1(R = fill(10, 3)) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {114, 10})));
        Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {66, -22})));
        Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
          Placement(transformation(extent = {{56, -58}, {76, -38}})));
        Modelica.Blocks.Sources.BooleanStep close1[3](startTime = 0.1*ones(3)) annotation(
          Placement(transformation(extent = {{10, 36}, {30, 56}})));
        Modelica.Electrical.Polyphase.Ideal.CloserWithArc switch(Ron = fill(1e-5, 3), Goff = fill(1e-5, 3), V0 = fill(30, 3), dVdt = fill(10000, 3), Vmax = fill(60, 3)) annotation(
          Placement(transformation(extent = {{36, 16}, {56, 36}})));
      equation
        connect(inductor.plug_n, resistor.plug_p) annotation(
          Line(points = {{-32, 24}, {-14, 24}, {-14, 18}}, color = {0, 0, 255}));
        connect(sineVoltage.plug_n, resistor.plug_n) annotation(
          Line(points = {{-114, -2}, {-114, -6}, {-14, -6}, {-14, -2}}, color = {0, 0, 255}));
        connect(star.plug_p, resistor.plug_n) annotation(
          Line(points = {{-62, -14}, {-62, -6}, {-14, -6}, {-14, -2}}, color = {0, 0, 255}));
        connect(star.pin_n, ground.p) annotation(
          Line(points = {{-62, -34}, {-62, -40}}, color = {0, 0, 255}));
        connect(inductor.plug_p, closerArc.nPlug) annotation(
          Line(points = {{-52, 24}, {-72, 24}}, color = {0, 0, 255}));
        connect(sineVoltage.plug_p, closerArc.pPlug) annotation(
          Line(points = {{-114, 18}, {-114, 24}, {-84, 24}}, color = {0, 0, 255}));
        connect(close.y, closerArc.u) annotation(
          Line(points = {{-97, 42}, {-80, 42}, {-80, 31}}, color = {255, 0, 255}));
        connect(inductor1.plug_n, resistor1.plug_p) annotation(
          Line(points = {{96, 26}, {114, 26}, {114, 20}}, color = {0, 0, 255}));
        connect(sineVoltage1.plug_n, resistor1.plug_n) annotation(
          Line(points = {{14, 0}, {14, -4}, {114, -4}, {114, 0}}, color = {0, 0, 255}));
        connect(star1.plug_p, resistor1.plug_n) annotation(
          Line(points = {{66, -12}, {66, -4}, {114, -4}, {114, 0}}, color = {0, 0, 255}));
        connect(star1.pin_n, ground1.p) annotation(
          Line(points = {{66, -32}, {66, -38}}, color = {0, 0, 255}));
        connect(inductor1.plug_p, switch.plug_n) annotation(
          Line(points = {{76, 26}, {56, 26}}, color = {0, 0, 255}));
        connect(sineVoltage1.plug_p, switch.plug_p) annotation(
          Line(points = {{14, 20}, {14, 26}, {36, 26}}, color = {0, 0, 255}));
        connect(close1.y, switch.control) annotation(
          Line(points = {{31, 46}, {46, 46}, {46, 38}}, color = {255, 0, 255}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false)),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-80, -60}, {80, 60}})),
          experiment(StopTime = 0.2, __Dymola_Algorithm = "Dassl"));
      end TestArcQuenchClose;

      model CloserArc
        Modelica.Blocks.Interfaces.BooleanInput u annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -78}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-20, 70})));
        Modelica.Blocks.Sources.RealExpression resVal_[3](y = fill(resVal, 3)) annotation(
          Placement(transformation(extent = {{-40, 14}, {-20, 34}})));
        final constant Real e = Modelica.Math.exp(1.0);
        parameter Modelica.Units.SI.Time tQuench = 10e-3 "Closing time";
        parameter Modelica.Units.SI.Resistance rMin = 1e-5 "Closed arc resistance ";
        parameter Modelica.Units.SI.Resistance rMax = 1e5 "Open arc resistance ";
        Modelica.Units.SI.Resistance resVal "Actual arc resistance ";
        Modelica.Units.SI.Time tStart(start = 1e99, fixed = true) "Initial closing time";
        Modelica.Electrical.Polyphase.Basic.VariableResistor resistor annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug pPlug annotation(
          Placement(transformation(extent = {{-70, -10}, {-50, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.NegativePlug nPlug annotation(
          Placement(transformation(extent = {{50, -10}, {70, 10}})));
      equation
        when u then
          tStart = time;
        end when;
        if time < tStart then
          resVal = rMax;
        else
          if time < tStart + tQuench then
            resVal = rMin + (rMax - rMin)*e^(-((time - tStart)/(0.1*tQuench))) "resVal exponentially decreases: e^(-t/tau)  ---->  0.1 * tQuench = tau";
//      resVal = rMax - (rMax - rMin)*((time - tStart)/tQuench) "linear";
          else
            resVal = rMin;
          end if;
        end if;
        connect(resistor.plug_n, nPlug) annotation(
          Line(points = {{10, 0}, {60, 0}}, color = {0, 0, 255}));
        connect(pPlug, resistor.plug_p) annotation(
          Line(points = {{-60, 0}, {-10, 0}}, color = {0, 0, 255}));
        connect(resVal_.y, resistor.R) annotation(
          Line(points = {{-19, 24}, {0, 24}, {0, 12}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-60, -60}, {60, 60}}), graphics = {Line(points = {{14, 40}}, color = {28, 108, 200}), Line(points = {{34, 26}, {14, 14}, {36, 0}, {14, -12}, {36, -26}, {14, -36}}, color = {238, 46, 47}), Line(points = {{34, 26}, {14, 36}}, color = {238, 46, 47}), Line(points = {{-60, 0}, {-22, 0}, {-2, 18}}, color = {28, 108, 200}), Line(points = {{40, 0}, {64, 0}}, color = {28, 108, 200}), Line(points = {{-4, 6}, {-4, 0}, {16, 0}}, color = {28, 108, 200}), Line(points = {{86, 42}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{-20, 54}, {-20, 8}}, color = {255, 0, 255}, pattern = LinePattern.Dash, thickness = 0.25), Line(points = {{-80, 42}}, color = {0, 0, 0}, thickness = 0.25), Text(extent = {{-80, -52}, {82, -82}}, textColor = {0, 0, 255}, textString = "%name")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-60, -60}, {60, 60}}), graphics = {Text(origin = {10, 43}, extent = {{-40, 3}, {40, -3}}, horizontalAlignment = TextAlignment.Left, textString = "resVal computed in the code", textColor = {0, 0, 0})}),
          Documentation(info = "<html>
<p>Simulates arc quench in a breaker.</p>
<p>When the input signal becomes ON, arc resistance grows exponentially from rMin to rMax in time tQuench.</p>
<p>the value of rMin should be very small (simulates the contacts of a closed breaker), rMax very large (simulates the open contacts gap resistance).</p>
<p>This resistance growth causes current to go to nearly zero, i.e. the breaker to open.</p>
</html>"));
      end CloserArc;

      model BreakerArc
        Modelica.Blocks.Interfaces.BooleanInput u annotation(
          Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {0, -78}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-20, 70})));
        Modelica.Blocks.Sources.RealExpression resVal_[3](y = fill(resVal, 3)) annotation(
          Placement(transformation(extent = {{-40, 14}, {-20, 34}})));
        parameter Modelica.Units.SI.Time tQuench = 10e-3 "Opening time";
        parameter Modelica.Units.SI.Resistance rMin = 1e-5 "Closed arc resistance ";
        parameter Modelica.Units.SI.Resistance rMax = 1e8 "Open arc resistance ";
        Modelica.Units.SI.Resistance resVal "Actual arc resistance ";
        Modelica.Units.SI.Time tStart(start = 1e99, fixed = true) "Initial opening time";
        Modelica.Electrical.Polyphase.Basic.VariableResistor resistor annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.PositivePlug pPlug annotation(
          Placement(transformation(extent = {{-70, -10}, {-50, 10}})));
        Modelica.Electrical.Polyphase.Interfaces.NegativePlug nPlug annotation(
          Placement(transformation(extent = {{50, -10}, {70, 10}})));
      equation
        when u then
          tStart = time;
        end when;
        if time < tStart then
          resVal = rMin;
        else
          if time < tStart + tQuench then
            resVal = rMin + (rMax - rMin)*((time - tStart)/tQuench)^2;
          else
            resVal = rMax;
          end if;
        end if;
        connect(resistor.plug_n, nPlug) annotation(
          Line(points = {{10, 0}, {60, 0}}, color = {0, 0, 255}));
        connect(pPlug, resistor.plug_p) annotation(
          Line(points = {{-60, 0}, {-10, 0}}, color = {0, 0, 255}));
        connect(resVal_.y, resistor.R) annotation(
          Line(points = {{-19, 24}, {0, 24}, {0, 12}}, color = {0, 0, 127}));
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-60, -60}, {60, 60}}), graphics = {Line(points = {{14, 40}}, color = {28, 108, 200}), Line(points = {{34, 26}, {14, 14}, {36, 0}, {14, -12}, {36, -26}, {14, -36}}, color = {238, 46, 47}), Line(points = {{34, 26}, {14, 36}}, color = {238, 46, 47}), Line(points = {{-60, 0}, {-22, 0}, {-2, 18}}, color = {28, 108, 200}), Line(points = {{40, 0}, {64, 0}}, color = {28, 108, 200}), Line(points = {{-4, 6}, {-4, 0}, {16, 0}}, color = {28, 108, 200}), Line(points = {{86, 42}}, color = {0, 0, 0}, thickness = 0.5), Line(points = {{-20, 54}, {-20, 8}}, color = {255, 0, 255}, pattern = LinePattern.Dash, thickness = 0.25), Line(points = {{-80, 42}}, color = {0, 0, 0}, thickness = 0.25), Text(extent = {{-80, -52}, {82, -82}}, textColor = {0, 0, 255}, textString = "%name")}),
          Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-60, -60}, {60, 60}}), graphics = {Text(origin = {10, 43}, extent = {{-40, 3}, {40, -3}}, horizontalAlignment = TextAlignment.Left, textString = "resVal computed in the code", textColor = {0, 0, 0})}),
          Documentation(info = "<html>
<p>Simulates arc quench in a breaker.</p>
<p>When the input signal becomes ON, arc resistance grows exponentially from rMin to rMax in time tQuench.</p>
<p>the value of rMin should be very small (simulates the contacts of a closed breaker), rMax very large (simulates the open contacts gap resistance).</p>
<p>This resistance growth causes current to go to nearly zero, i.e. the breaker to open.</p>
</html>"));
      end BreakerArc;
    end TestsBreakerCloser;
  end Electric;

  package PaperModels
    extends Modelica.Icons.ExamplesPackage;

    model case0_Figure10 "only grid-following VSCs -> black-out"
      import Modelica.Constants.pi;
      Real V_rms;
      VSC_GFL_GFM.Electric.DistortedGrid gridMV(ampl = 20000*sqrt(2/3), R = 0.072, L = 6e-3) annotation(
        Placement(transformation(origin = {-8, 0}, extent = {{136, -28}, {156, -8}})));
      Modelica.Electrical.Polyphase.Sensors.AronSensor pGrid annotation(
        Placement(transformation(origin = {94, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qGrid annotation(
        Placement(transformation(origin = {114, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(transformation(origin = {-8, 0}, extent = {{136, -60}, {156, -40}})));
      VSC_GFL_GFM.Electric.MV.ImTransfDy imTransfDy(Sbase = 30e3) annotation(
        Placement(transformation(origin = {8, 8}, extent = {{-92, -8}, {-72, 12}})));
      VSC_GFL_GFM.Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(origin = {34, 18}, extent = {{-10, -4}, {10, 4}}, rotation = -90)));
      VSC_GFL_GFM.Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, origin = {-14, 36})));
      VSC_GFL_GFM.Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, origin = {-54, 36})));
      VSC_GFL_GFM.Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, origin = {-94, 36})));
      Modelica.Blocks.Sources.Constant PQrefVsc1[2](k = {0.6, 0.3}) annotation(
        Placement(transformation(origin = {-46, -28}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
      Modelica.Blocks.Sources.Constant PQrefVsc2[2](k = {-0.1, 0}) annotation(
        Placement(transformation(origin = {-126, -28}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
      VSC_GFL_GFM.Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 10e3) annotation(
        Placement(transformation(origin = {-8, -6}, extent = {{4, 6}, {24, 26}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(origin = {34, -18}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {34, -42}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Modelica.Blocks.Sources.BooleanStep goToIsland(each startTime = 2) annotation(
        Placement(transformation(origin = {-26, 0}, extent = {{106, 24}, {86, 44}})));
      VSC_GFL_GFM.Electric.MV.GflBesCTransfDy VSC1 annotation(
        Placement(transformation(origin = {2, 8}, extent = {{-46, -8}, {-26, 12}})));
      VSC_GFL_GFM.Electric.MV.GflBesCTransfDy VSC2(sNom = 150e3) annotation(
        Placement(transformation(origin = {10, -10}, extent = {{-134, 10}, {-114, 30}})));
      VSC_GFL_GFM.Electric.TestsBreakerCloser.BreakerArc breaker annotation(
        Placement(transformation(origin = {-20, 0}, extent = {{66, -8}, {78, 4}})));
      Modelica.Blocks.Logical.TerminateSimulation terminateSimulation(condition = (V_rms > 500) or (V_rms < 50), terminationText = "Voltage out of limits") annotation(
        Placement(transformation(extent = {{94, 46}, {154, 52}})));
    equation
      V_rms = loadRLTransf.voltageRMSSensor.V;
      connect(pGrid.plug_p, qGrid.plug_n) annotation(
        Line(points = {{104, -2}, {104, -2}}, color = {0, 0, 255}));
      connect(qGrid.plug_p, gridMV.plug) annotation(
        Line(points = {{124, -2}, {138, -2}, {138, -8.2}, {138.2, -8.2}}, color = {0, 0, 255}));
      connect(gridMV.pin, ground1.p) annotation(
        Line(points = {{138, -28}, {138, -40}}, color = {0, 0, 255}));
      connect(cable1.positivePlug, cable2.negativePlug) annotation(
        Line(points = {{34, 28}, {34, 36}, {-4, 36}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{-24, 36}, {-44, 36}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable3.positivePlug) annotation(
        Line(points = {{-74.2, 20}, {-74.2, 36}, {-64, 36}}, color = {0, 0, 255}));
      connect(cable3.positivePlug, cable4.negativePlug) annotation(
        Line(points = {{-64, 36}, {-84, 36}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{34, -32}, {34, -28}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{34, -8}, {34, 8}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, VSC1.plugIn) annotation(
        Line(points = {{-24, 36}, {-34, 36}, {-34, 20}}, color = {0, 0, 255}));
      connect(cable4.positivePlug, VSC2.plugIn) annotation(
        Line(points = {{-104, 36}, {-114, 36}, {-114, 20}}, color = {0, 0, 255}));
      connect(VSC2.pqRef, PQrefVsc2.y) annotation(
        Line(points = {{-126, 2.4}, {-126, -19.2}}, color = {0, 0, 127}));
      connect(loadRLTransf.plugIn, cable2.negativePlug) annotation(
        Line(points = {{6, 20}, {6, 36}, {-4, 36}}, color = {0, 0, 255}));
      connect(VSC1.pqRef, PQrefVsc1.y) annotation(
        Line(points = {{-46, 2.4}, {-46, -19.2}}, color = {0, 0, 127}));
      connect(breaker.nPlug, pGrid.plug_n) annotation(
        Line(points = {{58, -2}, {84, -2}}, color = {0, 0, 255}));
      connect(breaker.pPlug, cable1.negativePlug) annotation(
        Line(points = {{46, -2}, {34, -2}, {34, 8}}, color = {0, 0, 255}));
      connect(goToIsland.y, breaker.u) annotation(
        Line(points = {{59, 34}, {50, 34}, {50, 5}}, color = {255, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-160, -60}, {160, 60}})),
        experiment(StartTime = -3, StopTime = 10, Interval = 0.000433333, Tolerance = 1e-06),
        Documentation(info = "<html>
<p>the micro-grid (VSC1 + VSC2 + LoadRL + induction motor) is generating active and reactive power, more than its demand (little P and Q exports when connected to external grid)</p>
<p>however, when switching to island mode the grid following VSCs are not able to sustain the micro-grid -&gt; black-out</p>
<ul>
<li>100 kVA VSC1 grid-following (e.g. photovoltaic) generating&nbsp;60 kW and 30 kvar</li>
<li>150 kVA VSC2 grid-following (vehicle charging point) with connected a BEV charging at 15 kW</li>
<li>induction 3ph motor consuming 30 kVA</li>
<li>RL load,&nbsp;10 kW&nbsp;with cos fi =0.9</li>
</ul>
<p><br>simulation is stopped when RMS Voltage measured at LoadRL is &lt;50V or &gt;500V</p>
</html>"),
        Icon(coordinateSystem(extent = {{-140, -140}, {300, 140}})));
    end case0_Figure10;

    model case1_GFMdroop_Figure11
      import Modelica.Constants.pi;
      import Modelica.Constants.inf;
      Electric.DistortedGrid gridMV(ampl = 20000*sqrt(2/3), R = 0.072, L = 6e-3) annotation(
        Placement(transformation(extent = {{136, -22}, {156, -2}})));
      Modelica.Electrical.Polyphase.Sensors.AronSensor pGrid annotation(
        Placement(transformation(origin = {102, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qGrid annotation(
        Placement(transformation(origin = {126, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(transformation(extent = {{136, -50}, {156, -30}})));
      Electric.MV.ImTransfDy imTransfDy(Sbase = 30e3) annotation(
        Placement(transformation(extent = {{-94, 4}, {-74, 24}})));
      Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = -90, origin = {42, 20})));
      Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-14, 42})));
      Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-60, 42})));
      Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-106, 42})));
      Modelica.Blocks.Sources.Constant PQrefVsc1[2](k = {-0.6, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-59, -11})));
      Modelica.Blocks.Sources.Constant PQrefVsc2[2](k = {-2/3, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-147, -11})));
      Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 50e3) annotation(
        Placement(transformation(extent = {{-2, 4}, {18, 24}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {42, -18})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {42, -38}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Electric.MV.GfdBesCTransfDy VSC1(pGain = 0.06, qGain = 0.04) annotation(
        Placement(transformation(extent = {{-48, 4}, {-28, 24}})));
      Electric.MV.GfdBesCTransfDy VSC2(sNom = 150e3) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-128, 14})));
  Modelica.Blocks.Sources.BooleanStep goToIsland(each startTime = 2) annotation(
        Placement(transformation(origin = {-10, 0}, extent = {{106, 24}, {86, 44}})));
  VSC_GFL_GFM.Electric.TestsBreakerCloser.BreakerArc breaker annotation(
        Placement(transformation(origin = {-8, 6}, extent = {{66, -8}, {78, 4}})));
    equation
      connect(pGrid.plug_p, qGrid.plug_n) annotation(
        Line(points = {{112, 4}, {116, 4}}, color = {0, 0, 255}));
      connect(qGrid.plug_p, gridMV.plug) annotation(
        Line(points = {{136, 4}, {146, 4}, {146, -2.2}, {146.2, -2.2}}, color = {0, 0, 255}));
      connect(gridMV.pin, ground1.p) annotation(
        Line(points = {{146, -22}, {146, -30}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{-24, 42}, {-50, 42}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable3.positivePlug) annotation(
        Line(points = {{-84.2, 24}, {-84.2, 42}, {-70, 42}}, color = {0, 0, 255}));
      connect(cable3.positivePlug, cable4.negativePlug) annotation(
        Line(points = {{-70, 42}, {-96, 42}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{42, -28}, {42, -28}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{42, -8}, {42, 10}}, color = {0, 0, 255}));
      connect(VSC1.plugIn, cable2.positivePlug) annotation(
        Line(points = {{-38, 24}, {-38, 42}, {-24, 42}}, color = {0, 0, 255}));
      connect(VSC2.plugIn, cable4.positivePlug) annotation(
        Line(points = {{-128, 24}, {-128, 42}, {-116, 42}}, color = {0, 0, 255}));
      connect(VSC2.pqRef, PQrefVsc2.y) annotation(
        Line(points = {{-140, 6.4}, {-140, 6}, {-147, 6}, {-147, -3.3}}, color = {0, 0, 127}));
      connect(VSC1.pqRef, PQrefVsc1.y) annotation(
        Line(points = {{-50, 6.4}, {-59, 6.4}, {-59, -3.3}}, color = {0, 0, 127}));
      connect(cable2.negativePlug, cable1.positivePlug) annotation(
        Line(points = {{-4, 42}, {42, 42}, {42, 30}}, color = {0, 0, 255}));
      connect(loadRLTransf.plugIn, cable1.positivePlug) annotation(
        Line(points = {{8, 24}, {8, 42}, {42, 42}, {42, 30}}, color = {0, 0, 255}));
  connect(breaker.pPlug, cable1.negativePlug) annotation(
        Line(points = {{58, 4}, {42, 4}, {42, 10}}, color = {0, 0, 255}));
  connect(breaker.nPlug, pGrid.plug_n) annotation(
        Line(points = {{70, 4}, {92, 4}}, color = {0, 0, 255}));
  connect(breaker.u, goToIsland.y) annotation(
        Line(points = {{62, 12}, {62, 34}, {76, 34}}, color = {255, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-160, -60}, {160, 60}})),
        experiment(StartTime = -3, StopTime = 10, Interval = 0.000433333, Tolerance = 1e-06),
        Documentation(info = "<html>
<p>the micro-grid (VSC1 + VSC2 + LoadRL + induction motor) is a net load for the external grid.</p>
<p>however, when switching to island mode the grid forming VSCs are able to sustain the micro-grid.</p>
<ul>
<li>100 kVA VSC1 grid-forming (vehicle charging point) with a connected BEV charging at 60 kW, enabled V2G control strategy</li>
<li>150 kVA VSC2 grid-forming (vehicle charging point) with a connected BEV charging at 100 kW, enabled V2G control strategy</li>
<li>induction 3ph motor consuming 30 kVA</li>
<li>RL load, 50 kW with cos fi =0.9</li>
</ul>
<p><br>VSC1 and VSC2 equipped with <b>GFM-droop</b> control strategy and slightly different P and Q droop coefficients (0.05-0.05 and 0.06-0.04)</p>
</html>"),
        Icon(coordinateSystem(extent = {{-160, -60}, {160, 60}})));
    end case1_GFMdroop_Figure11;

    model case1_GFMVSM_Figure12
      import Modelica.Constants.pi;
      Electric.DistortedGrid gridMV(startF2Time = 1000, ampl = 20000*sqrt(2/3), startHTime = 1000, R = 0.072, L = 6e-3) annotation(
        Placement(transformation(origin = {-142, 66}, extent = {{276, -94}, {296, -74}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.AronSensor pGrid annotation(
        Placement(transformation(origin = {104, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qGrid annotation(
        Placement(transformation(origin = {124, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Electric.MV.ImTransfDy imTransfDy(Sbase = 30e3) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-80, 10})));
      Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = -90, origin = {50, 22})));
      Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {0, 40})));
      Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-48, 40})));
      Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-106, 40})));
      Modelica.Blocks.Sources.Constant pqRefVsc1[2](k = {-0.6, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-47, -17})));
      Modelica.Blocks.Sources.Constant pqRefVsc2[2](k = {-2/3, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-145, -19})));
      Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 50e3) annotation(
        Placement(transformation(extent = {{12, 0}, {32, 20}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {50, -12})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {50, -32}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Electric.MV.VsmBesCTransfDy VSC1(pGain = 0.06, qGain = 0.04) annotation(
        Placement(transformation(origin = {-26, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Electric.MV.VsmBesCTransfDy VSC2(sNom = 150e3) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-128, 12})));
      Modelica.Electrical.Analog.Basic.Ground ground5 annotation(
        Placement(transformation(origin = {144, -42}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.BooleanStep goToIsland(each startTime = 2) annotation(
        Placement(transformation(origin = {4, -90}, extent = {{106, 24}, {86, 44}})));
  VSC_GFL_GFM.Electric.TestsBreakerCloser.BreakerArc breaker annotation(
        Placement(transformation(origin = {2, 2}, extent = {{66, 8}, {78, -4}})));
    equation
      connect(pGrid.plug_p, qGrid.plug_n) annotation(
        Line(points = {{114, 2}, {114, 2}}, color = {0, 0, 255}));
      connect(qGrid.plug_p, gridMV.plug) annotation(
        Line(points = {{134, 2}, {144.2, 2}, {144.2, -8.2}}, color = {0, 0, 255}));
      connect(cable1.positivePlug, cable2.negativePlug) annotation(
        Line(points = {{50, 32}, {50, 40}, {10, 40}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{-10, 40}, {-38, 40}}, color = {0, 0, 255}));
      connect(cable3.positivePlug, cable4.negativePlug) annotation(
        Line(points = {{-58, 40}, {-96, 40}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{50, -22}, {50, -22}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{50, -2}, {50, 12}}, color = {0, 0, 255}));
      connect(VSC1.plugIn, cable2.positivePlug) annotation(
        Line(points = {{-26, 21.8}, {-26, 40}, {-10, 40}}, color = {0, 0, 255}));
      connect(VSC2.plugIn, cable4.positivePlug) annotation(
        Line(points = {{-128, 21.8}, {-128, 40}, {-116, 40}}, color = {0, 0, 255}));
      connect(loadRLTransf.plugIn, cable2.negativePlug) annotation(
        Line(points = {{22, 20}, {22, 40}, {10, 40}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable4.negativePlug) annotation(
        Line(points = {{-80.2, 20}, {-80.2, 40}, {-96, 40}}, color = {0, 0, 255}));
      connect(gridMV.pin, ground5.p) annotation(
        Line(points = {{144, -28}, {144, -32}}, color = {0, 0, 255}));
      connect(VSC2.pqRef, pqRefVsc2.y) annotation(
        Line(points = {{-140, 4.4}, {-140, 4}, {-145, 4}, {-145, -11.3}}, color = {0, 0, 127}));
      connect(VSC1.pqRef, pqRefVsc1.y) annotation(
        Line(points = {{-38, 4.4}, {-38, 4}, {-46, 4}, {-46, -9.3}, {-47, -9.3}}, color = {0, 0, 127}));
  connect(breaker.nPlug, pGrid.plug_n) annotation(
        Line(points = {{80, 4}, {94, 4}, {94, 2}}, color = {0, 0, 255}));
  connect(breaker.pPlug, cable1.negativePlug) annotation(
        Line(points = {{68, 4}, {50, 4}, {50, 12}}, color = {0, 0, 255}));
  connect(breaker.u, goToIsland.y) annotation(
        Line(points = {{72, -2}, {72, -56}, {89, -56}}, color = {255, 0, 255}));
  connect(VSC1.mode, goToIsland.y) annotation(
        Line(points = {{-14, 4}, {0, 4}, {0, -56}, {90, -56}}, color = {255, 0, 255}));
  connect(VSC2.mode, goToIsland.y) annotation(
        Line(points = {{-116, 4}, {-108, 4}, {-108, -56}, {90, -56}}, color = {255, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-160, -60}, {160, 60}})),
        experiment(StartTime = -3, StopTime = 10, Interval = 0.000433333, Tolerance = 1e-06),
        Documentation(info = "<html>
<p>the micro-grid (VSC1 + VSC2 + LoadRL + induction motor) is a net load for the external grid.</p>
<p>however, when switching to island mode the grid forming VSCs are able to sustain the micro-grid.</p>
<ul>
<li>100 kVA VSC1 grid-forming (vehicle charging point) with a connected BEV charging at 60 kW, enabled V2G control strategy</li>
<li>150 kVA VSC2 grid-forming (vehicle charging point) with a connected BEV charging at 100 kW, enabled V2G control strategy</li>
<li>induction 3ph motor consuming 30 kVA</li>
<li>RL load, 50 kW with cos fi =0.9</li>
</ul>
<p><br>VSC1 and VSC2 equipped with <b>GFM-VSM</b> control strategy and slightly different P and Q droop coefficients (0.05-0.05 and 0.06-0.04)</p>
</html>"),
        Icon(coordinateSystem(extent = {{-160, -60}, {180, 60}})));
    end case1_GFMVSM_Figure12;

    model case2_GFMdroop_Figure13
      import Modelica.Constants.pi;
      Electric.DistortedGrid gridMV(ampl = 20000*sqrt(2/3), R = 0.072, L = 6e-3) annotation(
        Placement(transformation(extent = {{134, -24}, {154, -4}})));
      Modelica.Electrical.Polyphase.Sensors.AronSensor pGrid annotation(
        Placement(transformation(origin = {106, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qGrid annotation(
        Placement(transformation(origin = {126, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(transformation(extent = {{134, -48}, {154, -28}})));
      Electric.MV.ImTransfDy imTransfDy(Sbase = 30e3) annotation(
        Placement(transformation(extent = {{-92, 20}, {-72, 40}})));
      Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = -90, origin = {52, 34})));
      Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-8, 62})));
      Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-58, 62})));
      Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-106, 62})));
      Modelica.Blocks.Sources.Constant PQrefVsc1[2](k = {0.9, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-47, 5})));
      Modelica.Blocks.Sources.Constant PQrefVsc2[2](k = {-1/2, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-145, 1})));
      Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 50e3) annotation(
        Placement(transformation(extent = {{8, 22}, {28, 42}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {52, -6})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {52, -34}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Electric.MV.GfdBesCTransfDy VSC2(sNom = 150e3) annotation(
        Placement(transformation(extent = {{-138, 22}, {-118, 42}})));
      Electric.MV.GflBesCTransfDy VSC1 annotation(
        Placement(transformation(extent = {{-40, 24}, {-20, 44}})));
  Modelica.Blocks.Sources.BooleanStep goToIsland(each startTime = 2) annotation(
        Placement(transformation(origin = {-4, 20}, extent = {{106, 24}, {86, 44}})));
  VSC_GFL_GFM.Electric.TestsBreakerCloser.BreakerArc breaker annotation(
        Placement(transformation(origin = {2, 20}, extent = {{66, -8}, {78, 4}})));
    equation
      connect(pGrid.plug_p, qGrid.plug_n) annotation(
        Line(points = {{116, 18}, {116, 18}}, color = {0, 0, 255}));
      connect(qGrid.plug_p, gridMV.plug) annotation(
        Line(points = {{136, 18}, {144.2, 18}, {144.2, -4.2}}, color = {0, 0, 255}));
      connect(gridMV.pin, ground1.p) annotation(
        Line(points = {{144, -24}, {144, -28}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{-18, 62}, {-48, 62}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable3.positivePlug) annotation(
        Line(points = {{-82.2, 40}, {-82.2, 62}, {-68, 62}}, color = {0, 0, 255}));
      connect(cable1.positivePlug, loadRLTransf.plugIn) annotation(
        Line(points = {{52, 44}, {52, 62}, {18, 62}, {18, 42}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{52, -24}, {52, -16}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{52, 4}, {52, 24}}, color = {0, 0, 255}));
      connect(cable4.positivePlug, VSC2.plugIn) annotation(
        Line(points = {{-116, 62}, {-128, 62}, {-128, 42}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, VSC1.plugIn) annotation(
        Line(points = {{-18, 62}, {-30, 62}, {-30, 44}}, color = {0, 0, 255}));
      connect(PQrefVsc2.y, VSC2.pqRef) annotation(
        Line(points = {{-145, 8.7}, {-145, 24.4}, {-140, 24.4}}, color = {0, 0, 127}));
      connect(PQrefVsc1.y, VSC1.pqRef) annotation(
        Line(points = {{-47, 12.7}, {-48, 12.7}, {-48, 26.4}, {-42, 26.4}}, color = {0, 0, 127}));
      connect(cable4.negativePlug, cable3.positivePlug) annotation(
        Line(points = {{-96, 62}, {-68, 62}}, color = {0, 0, 255}));
      connect(cable2.negativePlug, loadRLTransf.plugIn) annotation(
        Line(points = {{2, 62}, {18, 62}, {18, 42}}, color = {0, 0, 255}));
  connect(breaker.pPlug, cable1.negativePlug) annotation(
        Line(points = {{68, 18}, {52, 18}, {52, 24}}, color = {0, 0, 255}));
  connect(pGrid.plug_n, breaker.nPlug) annotation(
        Line(points = {{96, 18}, {80, 18}}, color = {0, 0, 255}));
  connect(goToIsland.y, breaker.u) annotation(
        Line(points = {{82, 54}, {72, 54}, {72, 26}}, color = {255, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-160, -60}, {160, 80}})),
        experiment(StartTime = -3, StopTime = 10, Interval = 0.000433333, Tolerance = 1e-06),
        Documentation(info = "<html>
<p>the micro-grid (VSC1 + VSC2 + LoadRL + induction motor) is a net load for the external grid.</p>
<p>however, when switching to island mode the grid forming VSC is able to sustain the micro-grid.</p>
<ul>
<li>100 kVA VSC1 grid-following (e.g. photovoltaic) generating 90 kW</li>
<li>150 kVA VSC2 grid-forming (vehicle charging point) with a connected BEV charging at 75 kW, enabled V1G control strategy</li>
<li>induction 3ph motor consuming 30 kVA</li>
<li>RL load, 50 kW with cos fi =0.9</li>
</ul>
<p><br>VSC2 equipped with <b>GFM-droop</b> control strategy </p>
</html>"));
    end case2_GFMdroop_Figure13;

    model case2_GFMVSM_Figure13
      import Modelica.Constants.pi;
      Electric.DistortedGrid gridMV(ampl = 20000*sqrt(2/3), R = 0.072, L = 6e-3) annotation(
        Placement(transformation(extent = {{158, -34}, {178, -14}})));
      Modelica.Electrical.Polyphase.Sensors.AronSensor pGrid annotation(
        Placement(transformation(origin = {126, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qGrid annotation(
        Placement(transformation(origin = {146, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(transformation(extent = {{158, -68}, {178, -48}})));
      Electric.MV.ImTransfDy imTransfDy(Sbase = 30e3) annotation(
        Placement(transformation(extent = {{-68, 0}, {-48, 20}})));
      Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = -90, origin = {72, 22})));
      Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {14, 42})));
      Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-42, 42})));
      Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-78, 42})));
      Modelica.Blocks.Sources.Constant PQrefVsc1[2](k = {0.9, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-33, -17})));
      Modelica.Blocks.Sources.Constant PQrefVsc2[2](k = {-1/2, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-127, -11})));
      Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 50e3) annotation(
        Placement(transformation(extent = {{28, 6}, {48, 26}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {72, -12})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {72, -36}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Electric.MV.VsmBesCTransfDy VSC2(sNom = 150e3) annotation(
        Placement(transformation(extent = {{-116, 0}, {-96, 20}})));
      Electric.MV.GflBesCTransfDy VSC1 annotation(
        Placement(transformation(extent = {{-20, 0}, {0, 20}})));
  Modelica.Blocks.Sources.BooleanStep goToIsland(each startTime = 2) annotation(
        Placement(transformation(origin = {26, -84}, extent = {{106, 24}, {86, 44}})));
  VSC_GFL_GFM.Electric.TestsBreakerCloser.BreakerArc breaker annotation(
        Placement(transformation(origin = {22, 4}, extent = {{66, 8}, {78, -4}}, rotation = -0)));
    equation
      connect(pGrid.plug_p, qGrid.plug_n) annotation(
        Line(points = {{136, 6}, {136, 6}}, color = {0, 0, 255}));
      connect(qGrid.plug_p, gridMV.plug) annotation(
        Line(points = {{156, 6}, {168, 6}, {168, -10}, {168.2, -10}, {168.2, -14.2}}, color = {0, 0, 255}));
      connect(gridMV.pin, ground1.p) annotation(
        Line(points = {{168, -34}, {168, -48}}, color = {0, 0, 255}));
      connect(cable1.positivePlug, cable2.negativePlug) annotation(
        Line(points = {{72, 32}, {72, 42}, {24, 42}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{4, 42}, {-32, 42}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable3.positivePlug) annotation(
        Line(points = {{-58.2, 20}, {-58.2, 42}, {-52, 42}}, color = {0, 0, 255}));
      connect(cable3.positivePlug, cable4.negativePlug) annotation(
        Line(points = {{-52, 42}, {-68, 42}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{72, -26}, {72, -22}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{72, -2}, {72, 12}}, color = {0, 0, 255}));
      connect(cable4.positivePlug, VSC2.plugIn) annotation(
        Line(points = {{-88, 42}, {-106, 42}, {-106, 19.8}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, VSC1.plugIn) annotation(
        Line(points = {{4, 42}, {-10, 42}, {-10, 20}}, color = {0, 0, 255}));
      connect(VSC2.pqRef, PQrefVsc2.y) annotation(
        Line(points = {{-118, 2.4}, {-118, 2}, {-122, 2}, {-122, -3.3}, {-127, -3.3}}, color = {0, 0, 127}));
      connect(VSC1.pqRef, PQrefVsc1.y) annotation(
        Line(points = {{-22, 2.4}, {-33, 2.4}, {-33, -9.3}}, color = {0, 0, 127}));
      connect(loadRLTransf.plugIn, cable2.negativePlug) annotation(
        Line(points = {{38, 26}, {40, 26}, {40, 42}, {24, 42}}, color = {0, 0, 255}));
  connect(breaker.pPlug, resistor.plug_p) annotation(
        Line(points = {{88, 6}, {72, 6}, {72, -2}}, color = {0, 0, 255}));
  connect(pGrid.plug_n, breaker.nPlug) annotation(
        Line(points = {{116, 6}, {100, 6}}, color = {0, 0, 255}));
  connect(breaker.u, goToIsland.y) annotation(
        Line(points = {{92, 0}, {92, -50}, {112, -50}}, color = {255, 0, 255}));
  connect(VSC2.mode, goToIsland.y) annotation(
        Line(points = {{-94, 2}, {-84, 2}, {-84, -50}, {112, -50}}, color = {255, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-140, -80}, {180, 60}})),
        experiment(StartTime = -3, StopTime = 10, Interval = 0.000433333, Tolerance = 1e-06),
        Documentation(info = "<html>
<p>the micro-grid (VSC1 + VSC2 + LoadRL + induction motor) is a net load for the external grid.</p>
<p>however, when switching to island mode the grid forming VSC is able to sustain the micro-grid.</p>
<ul>
<li>100 kVA VSC1 grid-following (e.g. photovoltaic) generating 90 kW</li>
<li>150 kVA VSC2 grid-forming (vehicle charging point) with a connected BEV charging at 75 kW, enabled V1G control strategy</li>
<li>induction 3ph motor consuming 30 kVA</li>
<li>RL load, 50 kW with cos fi =0.9</li>
</ul>
<p><br>VSC2 equipped with <b>GFM-VSM</b> control strategy </p>
</html>"),
        Icon(coordinateSystem(extent = {{-140, -140}, {300, 140}})));
    end case2_GFMVSM_Figure13;

    model case3_GFMdroop_Figure14
      import Modelica.Constants.pi;
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(transformation(extent = {{78, -62}, {98, -42}})));
      Electric.MV.ImTransfDy imTransfDy(Sbase = 15e3, wInit = 0) annotation(
        Placement(transformation(extent = {{-48, 6}, {-28, 26}})));
      Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = -90, origin = {88, 28})));
      Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {32, 44})));
      Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-14, 44})));
      Modelica.Blocks.Sources.Constant PQrefVsc1[2](k = {0, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-85, -23})));
      Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 30e3) annotation(
        Placement(transformation(extent = {{44, 6}, {64, 26}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {88, 2})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {88, -24}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Electric.MV.GfdBesCTransfDy VSC2(tConn = 1, sNom = 150e3) annotation(
        Placement(transformation(extent = {{-90, 4}, {-70, 24}})));
      Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-64, 44})));
      Electric.MV.GflBesCTransfDy VSC1(tConn = 2, uDc = 645) annotation(
        Placement(transformation(extent = {{0, 6}, {20, 26}})));
      Modelica.Blocks.Sources.Constant QrefVsc2(k = 0) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-17, -21})));
      Modelica.Blocks.Sources.Ramp PrefVsc2(height = 0.8, duration = 0.3, startTime = 3) annotation(
        Placement(transformation(extent = {{-44, -50}, {-30, -36}})));
    equation
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{22, 44}, {-4, 44}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable3.positivePlug) annotation(
        Line(points = {{-38.2, 26}, {-38.2, 44}, {-24, 44}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{88, -14}, {88, -8}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{88, 12}, {88, 18}}, color = {0, 0, 255}));
      connect(VSC2.pqRef, PQrefVsc1.y) annotation(
        Line(points = {{-92, 6.4}, {-92, -12}, {-85, -12}, {-85, -15.3}}, color = {0, 0, 127}));
      connect(cable2.negativePlug, cable1.positivePlug) annotation(
        Line(points = {{42, 44}, {88, 44}, {88, 38}}, color = {0, 0, 255}));
      connect(loadRLTransf.plugIn, cable1.positivePlug) annotation(
        Line(points = {{54, 26}, {54, 44}, {88, 44}, {88, 38}}, color = {0, 0, 255}));
      connect(cable3.positivePlug, cable4.negativePlug) annotation(
        Line(points = {{-24, 44}, {-54, 44}}, color = {0, 0, 255}));
      connect(cable4.positivePlug, VSC2.plugIn) annotation(
        Line(points = {{-74, 44}, {-80, 44}, {-80, 24}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, VSC1.plugIn) annotation(
        Line(points = {{22, 44}, {10, 44}, {10, 26}}, color = {0, 0, 255}));
      connect(QrefVsc2.y, VSC1.pqRef[2]) annotation(
        Line(points = {{-17, -13.3}, {-18, -13.3}, {-18, 8.9}, {-2, 8.9}}, color = {0, 0, 127}));
      connect(PrefVsc2.y, VSC1.pqRef[1]) annotation(
        Line(points = {{-29.3, -43}, {-29.3, -44}, {-2, -44}, {-2, 7.9}}, color = {0, 0, 127}));
      connect(star1.pin_n, ground1.p) annotation(
        Line(points = {{88, -34}, {88, -42}}, color = {0, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})),
        experiment(StopTime = 10, Interval = 0.0005, Tolerance = 1e-06, StartTime = 0),
        Documentation(info="<html>
<p>Typical black-start of MV grid portion, after a total black-out event.</p>
<p>After 1 s the VSC2-<b>GFM-droop</b> of a charging station with a BEV connected (V2G enabled), starts to energize the rest of the grid supplying active and reactive power to the loads, including motor inrush.</p>
<p>Then at t = 2s the VSC1-GFL interfacing a DC source (e.g. photovoltaic) starts its synchronization with the local grid and after 1s begins to inject active power. </p>
<ul>
<li>100 kVA VSC1 grid-following (e.g. photovoltaic) generating 80 kW</li>
<li>150 kVA VSC2 grid-forming (vehicle charging point) with a connected BEV, enabled V2G control strategy</li>
<li>induction 3ph motor consuming 15 kVA</li>
<li>RL load, 30 kW with cos fi =0.9</li>
</ul>
</html>"),
        Icon(coordinateSystem(extent = {{-100, -60}, {100, 60}})));
    end case3_GFMdroop_Figure14;

    model case4_GFMdroop_Figure15
      import Modelica.Constants.pi;
      Electric.DistortedGrid gridMV(startF2Time = 2, ampl = 20000*sqrt(2/3), ampl2 = 0.98*20000*sqrt(2/3), startV2Time = 5, R = 0.072, L = 6e-3) annotation(
        Placement(transformation(extent = {{108, -22}, {128, -2}})));
      Modelica.Electrical.Polyphase.Sensors.AronSensor pGrid annotation(
        Placement(transformation(origin = {74, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor qGrid annotation(
        Placement(transformation(origin = {98, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(transformation(extent = {{108, -50}, {128, -30}})));
      Electric.MV.ImTransfDy imTransfDy(Sbase = 30e3) annotation(
        Placement(transformation(extent = {{-94, 4}, {-74, 24}})));
      Electric.MV.CableRLMV cable1 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = -90, origin = {42, 20})));
      Electric.MV.CableRLMV cable2 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-14, 42})));
      Electric.MV.CableRLMV cable3 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-60, 42})));
      Electric.MV.CableRLMV cable4 annotation(
        Placement(transformation(extent = {{-10, -4}, {10, 4}}, rotation = 0, origin = {-106, 42})));
      Modelica.Blocks.Sources.Constant PQrefVsc1[2](k = {-0.6, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-59, -11})));
      Modelica.Blocks.Sources.Constant PQrefVsc2[2](k = {-2/3, 0}) annotation(
        Placement(transformation(extent = {{-7, -7}, {7, 7}}, rotation = 90, origin = {-147, -11})));
      Electric.MV.LoadRLTransfDy loadRLTransf(pNom = 50e3) annotation(
        Placement(transformation(extent = {{-2, 4}, {18, 24}})));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = 100e3*ones(3)) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {42, -18})));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(transformation(origin = {42, -38}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
      Electric.MV.GfdBesCTransfDy VSC1(pGain = 0.06, qGain = 0.04, secReg = 0) annotation(
        Placement(transformation(extent = {{-48, 4}, {-28, 24}})));
      Electric.MV.GfdBesCTransfDy VSC2(sNom = 150e3, secReg = 0) annotation(
        Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-128, 14})));
    equation
      connect(pGrid.plug_p, qGrid.plug_n) annotation(
        Line(points = {{84, 4}, {88, 4}}, color = {0, 0, 255}));
      connect(qGrid.plug_p, gridMV.plug) annotation(
        Line(points = {{108, 4}, {118, 4}, {118, -2.2}, {118.2, -2.2}}, color = {0, 0, 255}));
      connect(gridMV.pin, ground1.p) annotation(
        Line(points = {{118, -22}, {118, -30}}, color = {0, 0, 255}));
      connect(cable2.positivePlug, cable3.negativePlug) annotation(
        Line(points = {{-24, 42}, {-50, 42}}, color = {0, 0, 255}));
      connect(imTransfDy.plugIn, cable3.positivePlug) annotation(
        Line(points = {{-84.2, 24}, {-84.2, 42}, {-70, 42}}, color = {0, 0, 255}));
      connect(cable3.positivePlug, cable4.negativePlug) annotation(
        Line(points = {{-70, 42}, {-96, 42}}, color = {0, 0, 255}));
      connect(star1.plug_p, resistor.plug_n) annotation(
        Line(points = {{42, -28}, {42, -28}}, color = {0, 0, 255}));
      connect(resistor.plug_p, cable1.negativePlug) annotation(
        Line(points = {{42, -8}, {42, 10}}, color = {0, 0, 255}));
      connect(VSC1.plugIn, cable2.positivePlug) annotation(
        Line(points = {{-38, 24}, {-38, 42}, {-24, 42}}, color = {0, 0, 255}));
      connect(VSC2.plugIn, cable4.positivePlug) annotation(
        Line(points = {{-128, 24}, {-128, 42}, {-116, 42}}, color = {0, 0, 255}));
      connect(VSC2.pqRef, PQrefVsc2.y) annotation(
        Line(points = {{-140, 6.4}, {-140, 6}, {-147, 6}, {-147, -3.3}}, color = {0, 0, 127}));
      connect(VSC1.pqRef, PQrefVsc1.y) annotation(
        Line(points = {{-50, 6.4}, {-59, 6.4}, {-59, -3.3}}, color = {0, 0, 127}));
      connect(cable2.negativePlug, cable1.positivePlug) annotation(
        Line(points = {{-4, 42}, {42, 42}, {42, 30}}, color = {0, 0, 255}));
      connect(loadRLTransf.plugIn, cable1.positivePlug) annotation(
        Line(points = {{8, 24}, {8, 42}, {42, 42}, {42, 30}}, color = {0, 0, 255}));
      connect(pGrid.plug_n, cable1.negativePlug) annotation(
        Line(points = {{64, 4}, {42, 4}, {42, 10}}, color = {0, 0, 255}));
      annotation(
        Diagram(coordinateSystem(extent = {{-160, -60}, {140, 60}})),
        experiment(StartTime = -3, StopTime = 10, Interval = 0.000433333, Tolerance = 1e-06),
        Documentation(info = "<html>
<p>the micro-grid (VSC1 + VSC2 + LoadRL + induction motor) is a net load for the external grid.</p>
<ul>
<li>100 kVA VSC1 grid-forming (vehicle charging point) with a connected BEV charging at 60 kW, enabled V2G control strategy</li>
<li>150 kVA VSC2 grid-forming (vehicle charging point) with a connected BEV charging at 100 kW, enabled V2G control strategy</li>
<li>induction 3ph motor consuming 30 kVA</li>
<li>RL load, 50 kW with cos fi =0.9</li>
</ul>
<p><br>Between 2-3 s a ramp of frequency (-0.5 Hz respect nominal 50 Hz) is imposed from external main grid and the response of GFM-droop VSCs is a reduction of BEV charging power (V1G). </p>
<p>At t = 5 s a MV voltage drop (-2&percnt; respect nominal 20 kV) is imposed and results show how GFM-droop control reacts injecting reactive power. </p>
</html>"),
        Icon(coordinateSystem(extent = {{-160, -60}, {140, 60}})));
    end case4_GFMdroop_Figure15;
  end PaperModels;
  annotation(
    uses(Modelica(version = "4.0.0")));
end VSC_GFL_GFM;

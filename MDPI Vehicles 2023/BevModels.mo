package BevModels
  model Tesla
    Real timeM = time/60;
    Modelica.Units.SI.Energy enP2(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP1(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP0(start = 0, fixed = true);
    Modelica.Units.SI.Energy enDC(start = 0, fixed = true);
    Modelica.Units.SI.Energy enDCPos(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP1Pos(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP1Neg(start = 0, fixed = true);
    Modelica.Units.SI.Energy enLost(start = 0, fixed = true);
    Real effEle;
    Real effEle_I;
    Real effP1;
    Real effTot_I;
    Real effP2;
    Real effMech;
    Real effMech_I;
    Real lossPower_Tot;
    Real lossPower_Ele;
    Real lossPower_Mech;
    Real consumption(unit = "W.h/km");
    Real consumption_Pos(unit = "W.h/km");
    Modelica.Mechanics.Rotational.Components.IdealRollingWheel wheel(radius=
          data.radius) annotation (Placement(visible=true, transformation(
          origin={86,20},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Mechanics.Translational.Sensors.PowerSensor mP1 annotation (
      Placement(visible = true, transformation(origin={112,20},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Translational.Components.Mass mass(m = data.m) annotation (
      Placement(visible = true, transformation(origin={138,20},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Translational.Sensors.PowerSensor mP2 annotation (
      Placement(visible = true, transformation(origin={170,2},    extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor annotation (
      Placement(visible = true, transformation(origin={154,-4},   extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Rotational.Sensors.PowerSensor mP0 annotation (
      Placement(visible = true, transformation(origin={-44,20},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.Integrator toEnDrag annotation (
      Placement(visible = true, transformation(origin={130,54},    extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    EHPTlib.MapBased.OneFlange eleDrive(
      J=data.J_rotor,
      effTableName="effTable",
      mapsFileName="TeslaMaps.txt",
      mapsOnFile=true,
      powMax=data.MaxPower,
      tauMax=data.MaxTorque,
      uDcNom=data.motorVoltage,
      wMax=data.MaxOmega) annotation (Placement(visible=true, transformation(
          origin={-98,20},
          extent={{-10,-8},{10,10}},
          rotation=0)));
    EHPTlib.SupportModels.Miscellaneous.DragForce dragForce(Cx = data.Cx, S = data.S, fc = data.fc, m = data.m, rho = data.rho) annotation (
      Placement(visible = true, transformation(origin={170,-26},    extent = {{10, -10}, {-10, 10}}, rotation = -90)));
    Modelica.Blocks.Nonlinear.Limiter to_mP1Pos(uMax = 1e99, uMin = 0) annotation (
      Placement(visible = true, transformation(origin={126,-12},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter to_mP1Neg(uMax = 0, uMin = -1e99) annotation (
      Placement(visible = true, transformation(origin={82,-12},   extent = {{-10, 10}, {10, -10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.PowerSensor mPDC annotation (
      Placement(visible = true, transformation(origin={-128,36},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter to_mPDCPos(uMax = 1e99, uMin = 0) annotation (
      Placement(visible = true, transformation(origin={-164,58},    extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    EHPTlib.SupportModels.Miscellaneous.Batt1 battery(ECellMax = data.ECellMax, ECellMin = data.ECellMin, ICellMax = data.ICellMax, QCellNom = data.QCellNom, R0Cell = 0.02*data.ECellMax/data.ICellMax, SOCInit = 1, efficiency = 0.95, np = data.np, ns = data.ns) annotation (
      Placement(visible = true, transformation(origin={0,16},      extent = {{-174, -4}, {-154, 16}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation (
      Placement(visible = true, transformation(origin={-168,-10},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.Inertia wheels_differential(J = data.J_d + data.J_w) annotation (
      Placement(transformation(extent={{24,10},{44,30}})));
    Modelica.Mechanics.Rotational.Components.BearingFriction wheelBearing(tau_pos=[
          0,1.26; 1571,1.26]) annotation (Placement(visible=true,
          transformation(
          origin={60,20},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Mechanics.Rotational.Components.LossyGear lossyGear(ratio = data.gear1Ratio, lossTable = [0, 0.98, 0.98, 1.3, 1.3; 1571, 0.98, 0.98, 1.3, 1.3]) annotation (
      Placement(visible = true, transformation(origin={-18,20},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.LossyGear lossyDifferential(ratio = data.differentialRatio, lossTable = [0, 0.97, 0.97, 1.78, 1.78; 1571, 0.97, 0.97, 1.78, 1.78]) annotation (
      Placement(visible = true, transformation(extent={{-2,10},{18,30}},      rotation = 0)));
    Modelica.Mechanics.Rotational.Components.BearingFriction rotorBearing(tau_pos=[
          0,0.5; 1571,0.5]) annotation (Placement(visible=true, transformation(
          origin={-72,20},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Records.TeslaS60 data annotation (
      Placement(transformation(extent={{-72,44},{-52,64}})));
    EHPTlib.SupportModels.Miscellaneous.PropDriver
                              myPropDriver(
      CycleFileName="WLTC3.txt",
      extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
      k=data.kContr,
      yMax=data.MaxTorque)                                                                                                                                                                                                         annotation (
      Placement(transformation(extent={{-142,-16},{-122,4}})));
  equation
    der(enP2) = mP2.power;
    der(enP1) = mP1.power;
    der(enP1Pos) = to_mP1Pos.y;
    der(enP1Neg) = -to_mP1Neg.y;
    der(enP0) = mP0.power;
    der(enDC) = mPDC.power;
    der(enDCPos) = to_mPDCPos.y;
    der(enLost) = lossPower_Tot;
    effEle = enP0/(enDC + 0.01);
    effP1 = enP1/(enDC + 0.01);
    effP2 = enP2/(enDC + 0.01);
    effMech = enP1/(enP0 + 0.01);
    lossPower_Tot = mPDC.power - mP1.power;
    lossPower_Ele = mPDC.power - mP0.power;
    lossPower_Mech = mP0.power - mP1.power;
    consumption = mPDC.power/(3.600*der(mass.s) + 0.01);
    consumption_Pos = der(enDCPos)/(3.600*der(mass.s) + 0.01);
    if mass.a >= -0.01 then
      if abs(mPDC.power) <= 1e3 then
        effEle_I = 0;
//effMech_I = 0;
      elseif mP0.power > mPDC.power then
        effEle_I = 0;
//effMech_I = 0;
      else
        effEle_I = mP0.power/mPDC.power;
//effMech_I = mP1.power/mP0.power;
      end if;
    else
      if abs(mP0.power) <= 1e3 then
        effEle_I = 0;
//effMech_I = 0;
      elseif abs(mPDC.power) > abs(mP0.power) or sign(mP0.power) <> sign(mPDC.power) then
        effEle_I = 0;
//effMech_I = 0;
      else
        effEle_I = mPDC.power/mP0.power;
//effMech_I = mP0.power/mP1.power;
      end if;
    end if;
    if mass.a >= -0.01 then
      if abs(mP0.power) <= 1e3 then
        effMech_I = 0;
      elseif mP1.power > mP0.power then
        effMech_I = 0;
      else
        effMech_I = mP1.power/mP0.power;
      end if;
    else
      if abs(mP0.power) <= 1e3 then
        effMech_I = 0;
      elseif abs(mP0.power) > abs(mP1.power) or sign(mP0.power) <> sign(mP1.power) then
        effMech_I = 0;
      else
        effMech_I = mP0.power/mP1.power;
      end if;
    end if;
    if mass.a >= -0.01 then
      if abs(mPDC.power) <= 1e2 then
        effTot_I = 0;
      elseif mP1.power > mPDC.power then
        effTot_I = 0;
      else
        effTot_I = mP1.power/mPDC.power;
      end if;
    else
      if abs(mPDC.power) <= 1e2 then
        effTot_I = 0;
      elseif abs(mPDC.power) > abs(mP1.power) or sign(mPDC.power) <> sign(mP1.power) then
        effTot_I = 0;
      else
        effTot_I = mPDC.power/mP1.power;
      end if;
    end if;
    connect(wheel.flangeT, mP1.flange_a)
      annotation (Line(points={{96,20},{102,20}}, color={0,127,0}));
    connect(mP1.flange_b, mass.flange_a) annotation (
      Line(points={{122,20},{128,20}},      color = {0, 127, 0}));
    connect(mass.flange_b, mP2.flange_a) annotation (
      Line(points={{148,20},{170,20},{170,12}},        color = {0, 127, 0}));
    connect(speedSensor.flange, mass.flange_b) annotation (
      Line(points={{154,6},{154,20},{148,20}},         color = {0, 127, 0}));
    connect(mP2.power, toEnDrag.u) annotation (
      Line(points={{159,10},{158,10},{158,54},{142,54}},          color = {0, 0, 127}));
    connect(dragForce.flange, mP2.flange_b) annotation (
      Line(points={{170,-16},{170,-8}},      color = {0, 127, 0}));
    connect(mP1.power, to_mP1Neg.u) annotation (
      Line(points={{104,9},{104,-12},{94,-12}},       color = {0, 0, 127}));
    connect(mP1.power, to_mP1Pos.u) annotation (
      Line(points={{104,9},{104,-12},{114,-12}},       color = {0, 0, 127}));
    connect(mPDC.pc, mPDC.pv) annotation (
      Line(points={{-138,36},{-138,46},{-128,46}},        color = {0, 0, 255}));
    connect(mPDC.nc, eleDrive.pin_p) annotation (Line(points={{-118,36},{-112,
            36},{-112,24},{-108,24}}, color={0,0,255}));
    connect(mPDC.nv, eleDrive.pin_n) annotation (Line(points={{-128,26},{-128,
            16},{-108,16}}, color={0,0,255}));
    connect(to_mPDCPos.u, mPDC.power) annotation (
      Line(points={{-152,58},{-146,58},{-146,20},{-138,20},{-138,25}},            color = {0, 0, 127}));
    connect(mPDC.pc, battery.p) annotation (
      Line(points={{-138,36},{-152,36},{-152,28},{-154,28}},          color = {0, 0, 255}));
    connect(battery.n, eleDrive.pin_n)
      annotation (Line(points={{-153.9,16},{-108,16}}, color={0,0,255}));
    connect(battery.n, ground.p) annotation (
      Line(points={{-153.9,16},{-150,16},{-150,6},{-168,6},{-168,0}},              color = {0, 0, 255}));
    connect(wheels_differential.flange_b, wheelBearing.flange_a)
      annotation (Line(points={{44,20},{50,20}}, color={0,0,0}));
    connect(wheelBearing.flange_b, wheel.flangeR)
      annotation (Line(points={{70,20},{76,20}}, color={0,0,0}));
    connect(lossyGear.flange_b, lossyDifferential.flange_a) annotation (
      Line(points={{-8,20},{-2,20}},       color = {0, 0, 0}));
    connect(wheels_differential.flange_a, lossyDifferential.flange_b) annotation (
      Line(points={{24,20},{18,20}},      color = {0, 0, 0}));
    connect(eleDrive.flange_a, rotorBearing.flange_a)
      annotation (Line(points={{-88,20},{-82,20}}, color={0,0,0}));
    connect(rotorBearing.flange_b, mP0.flange_a)
      annotation (Line(points={{-62,20},{-54,20}}, color={0,0,0}));
    connect(mP0.flange_b, lossyGear.flange_a) annotation (
      Line(points={{-34,20},{-28,20}},      color = {0, 0, 0}));
    connect(eleDrive.tauRef, myPropDriver.tauRef) annotation (Line(points={{
            -109.4,20},{-116,20},{-116,-6},{-121,-6}}, color={0,0,127}));
    connect(myPropDriver.V, speedSensor.v) annotation (Line(points={{-132,-17.2},
            {-132,-40},{154,-40},{154,-15}},
                                           color={0,0,127}));
    annotation (
      Diagram(coordinateSystem(extent={{-180,-60},{180,80}})),
      experiment(StopTime = 1800, Interval = 0.760135, Tolerance = 1e-06, StartTime = 0),
      Icon(coordinateSystem(extent = {{-200, -60}, {180, 80}})));
  end Tesla;

  model Nissan
    Real timeM = time/60;
    Modelica.Units.SI.Energy enP2(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP1(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP0(start = 0, fixed = true);
    Modelica.Units.SI.Energy enDC(start = 0, fixed = true);
    Modelica.Units.SI.Energy enDCPos(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP1Pos(start = 0, fixed = true);
    Modelica.Units.SI.Energy enP1Neg(start = 0, fixed = true);
    Modelica.Units.SI.Energy enLost(start = 0, fixed = true);
    Real effEle;
    Real effEle_I;
    Real effP1;
    Real effTot_I;
    Real effP2;
    Real effMech;
    Real effMech_I;
    Real lossPower_Tot;
    Real lossPower_Ele;
    Real lossPower_Mech;
    Real consumption(unit = "W.h/km");
    Real consumption_Pos(unit = "W.h/km");
    Modelica.Mechanics.Rotational.Components.IdealRollingWheel wheel(radius=
          data.radius) annotation (Placement(visible=true, transformation(
          origin={82,26},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Mechanics.Translational.Sensors.PowerSensor mP1 annotation (
      Placement(visible = true, transformation(origin = {108, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Translational.Components.Mass mass(m = data.m) annotation (
      Placement(visible = true, transformation(origin = {134, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Translational.Sensors.PowerSensor mP2 annotation (
      Placement(visible = true, transformation(origin = {166, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor annotation (
      Placement(visible = true, transformation(origin = {150, 2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Rotational.Sensors.PowerSensor mP0 annotation (
      Placement(visible = true, transformation(origin = {-54, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.Integrator toEnDrag annotation (
      Placement(visible = true, transformation(origin = {126, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    EHPTlib.MapBased.OneFlange eleDrive(
      J=data.J_rotor,
      effTableName="effTable",
      mapsFileName="NissanMaps.txt",
      mapsOnFile=true,
      powMax=data.MaxPower,
      tauMax=data.MaxTorque,
      uDcNom=data.motorVoltage,
      wMax=data.MaxOmega) annotation (Placement(visible=true, transformation(
          origin={-108,26},
          extent={{-10,-8},{10,10}},
          rotation=0)));
    EHPTlib.SupportModels.Miscellaneous.DragForce dragForce(Cx = data.Cx, S = data.S, fc = data.fc, m = data.m, rho = data.rho) annotation (
      Placement(visible = true, transformation(origin = {166, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
    Modelica.Blocks.Nonlinear.Limiter to_mP1Pos(uMax = 1e99, uMin = 0) annotation (
      Placement(visible = true, transformation(origin = {122, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter to_mP1Neg(uMax = 0, uMin = -1e99) annotation (
      Placement(visible = true, transformation(origin = {78, -6}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
    Modelica.Electrical.Analog.Sensors.PowerSensor mPDC annotation (
      Placement(visible = true, transformation(origin = {-138, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Nonlinear.Limiter to_mPDCPos(uMax = 1e99, uMin = 0) annotation (
      Placement(visible = true, transformation(origin = {-176, 64}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    EHPTlib.SupportModels.Miscellaneous.Batt1 battery(ECellMax = data.ECellMax, ECellMin = data.ECellMin, ICellMax = data.ICellMax, QCellNom = data.QCellNom, R0Cell = 0.02*data.ECellMax/data.ICellMax, SOCInit = 1, efficiency = 0.95, np = data.np, ns = data.ns) annotation (
      Placement(visible = true, transformation(origin = {-12, 22}, extent = {{-174, -4}, {-154, 16}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation (
      Placement(visible = true, transformation(origin = {-178, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.Inertia wheels_differential(J = data.J_d + data.J_w) annotation (
      Placement(transformation(extent = {{20, 16}, {40, 36}})));
    Modelica.Mechanics.Rotational.Components.BearingFriction wheelBearing(tau_pos=[
          0,0.85; 1088,0.85]) annotation (Placement(visible=true,
          transformation(
          origin={56,26},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Mechanics.Rotational.Components.LossyGear lossyGear(ratio = data.gear1Ratio, lossTable = [0, 0.98, 0.98, 0.58, 0.58; 1088, 0.98, 0.98, 0.58, 0.58]) annotation (
      Placement(visible = true, transformation(origin = {-22, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.LossyGear lossyDifferential(ratio = data.differentialRatio, lossTable = [0, 0.97, 0.97, 0.72, 0.72; 1088, 0.97, 0.97, 0.72, 0.72]) annotation (
      Placement(visible = true, transformation(extent = {{-6, 16}, {14, 36}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.BearingFriction rotBearingFric(tau_pos = [0, 0.3; 1088, 0.3]) annotation (
      Placement(visible = true, transformation(origin = {-82, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Records.NissanLEAF data annotation (
      Placement(transformation(extent = {{-94, 48}, {-74, 68}})));
    EHPTlib.SupportModels.Miscellaneous.PropDriver propDriver(
      CycleFileName="WLTC3.txt",
      extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
      k=data.kContr,
      yMax=data.MaxTorque)
      annotation (Placement(transformation(extent={{-154,-14},{-134,6}})));
  equation
    der(enP2) = mP2.power;
    der(enP1) = mP1.power;
    der(enP1Pos) = to_mP1Pos.y;
    der(enP1Neg) = -to_mP1Neg.y;
    der(enP0) = mP0.power;
    der(enDC) = mPDC.power;
    der(enDCPos) = to_mPDCPos.y;
    der(enLost) = lossPower_Tot;
    effEle = enP0/(enDC + 0.01);
    effP1 = enP1/(enDC + 0.01);
    effP2 = enP2/(enDC + 0.01);
    effMech = enP1/(enP0 + 0.01);
    lossPower_Tot = mPDC.power - mP1.power;
    lossPower_Ele = mPDC.power - mP0.power;
    lossPower_Mech = mP0.power - mP1.power;
    consumption = mPDC.power/(3.600*der(mass.s) + 0.01);
    consumption_Pos = der(enDCPos)/(3.600*der(mass.s) + 0.01);
    if mass.a >= -0.01 then
      if abs(mPDC.power) <= 1e3 then
        effEle_I = 0;
//effMech_I = 0;
      elseif mP0.power > mPDC.power then
        effEle_I = 0;
//effMech_I = 0;
      else
        effEle_I = mP0.power/mPDC.power;
//effMech_I = mP1.power/mP0.power;
      end if;
    else
      if abs(mP0.power) <= 1e3 then
        effEle_I = 0;
//effMech_I = 0;
      elseif abs(mPDC.power) > abs(mP0.power) or sign(mP0.power) <> sign(mPDC.power) then
        effEle_I = 0;
//effMech_I = 0;
      else
        effEle_I = mPDC.power/mP0.power;
//effMech_I = mP0.power/mP1.power;
      end if;
    end if;
    if mass.a >= -0.01 then
      if abs(mP0.power) <= 1e3 then
        effMech_I = 0;
      elseif mP1.power > mP0.power then
        effMech_I = 0;
      else
        effMech_I = mP1.power/mP0.power;
      end if;
    else
      if abs(mP0.power) <= 1e3 then
        effMech_I = 0;
      elseif abs(mP0.power) > abs(mP1.power) or sign(mP0.power) <> sign(mP1.power) then
        effMech_I = 0;
      else
        effMech_I = mP0.power/mP1.power;
      end if;
    end if;
    if mass.a >= -0.01 then
      if abs(mPDC.power) <= 1e2 then
        effTot_I = 0;
      elseif mP1.power > mPDC.power then
        effTot_I = 0;
      else
        effTot_I = mP1.power/mPDC.power;
      end if;
    else
      if abs(mPDC.power) <= 1e2 then
        effTot_I = 0;
      elseif abs(mPDC.power) > abs(mP1.power) or sign(mPDC.power) <> sign(mP1.power) then
        effTot_I = 0;
      else
        effTot_I = mPDC.power/mP1.power;
      end if;
    end if;
    connect(wheel.flangeT, mP1.flange_a)
      annotation (Line(points={{92,26},{98,26}}, color={0,127,0}));
    connect(mP1.flange_b, mass.flange_a) annotation (
      Line(points = {{118, 26}, {124, 26}}, color = {0, 127, 0}));
    connect(mass.flange_b, mP2.flange_a) annotation (
      Line(points = {{144, 26}, {166, 26}, {166, 18}}, color = {0, 127, 0}));
    connect(speedSensor.flange, mass.flange_b) annotation (
      Line(points = {{150, 12}, {150, 26}, {144, 26}}, color = {0, 127, 0}));
    connect(mP2.power, toEnDrag.u) annotation (
      Line(points = {{155, 16}, {154, 16}, {154, 60}, {138, 60}}, color = {0, 0, 127}));
    connect(dragForce.flange, mP2.flange_b) annotation (
      Line(points = {{166, -10}, {166, -2}}, color = {0, 127, 0}));
    connect(mP1.power, to_mP1Neg.u) annotation (
      Line(points = {{100, 15}, {100, -6}, {90, -6}}, color = {0, 0, 127}));
    connect(mP1.power, to_mP1Pos.u) annotation (
      Line(points = {{100, 15}, {100, -6}, {110, -6}}, color = {0, 0, 127}));
    connect(mPDC.pc, mPDC.pv) annotation (
      Line(points = {{-148, 42}, {-148, 52}, {-138, 52}}, color = {0, 0, 255}));
    connect(mPDC.nc, eleDrive.pin_p) annotation (Line(points={{-128,42},{-122,
            42},{-122,30},{-118,30}}, color={0,0,255}));
    connect(mPDC.nv, eleDrive.pin_n) annotation (Line(points={{-138,32},{-138,
            22},{-118,22}}, color={0,0,255}));
    connect(to_mPDCPos.u, mPDC.power) annotation (
      Line(points = {{-164, 64}, {-156, 64}, {-156, 26}, {-148, 26}, {-148, 31}}, color = {0, 0, 127}));
    connect(mPDC.pc, battery.p) annotation (
      Line(points = {{-148, 42}, {-162, 42}, {-162, 34}, {-166, 34}}, color = {0, 0, 255}));
    connect(battery.n, eleDrive.pin_n)
      annotation (Line(points={{-165.9,22},{-118,22}}, color={0,0,255}));
    connect(battery.n, ground.p) annotation (
      Line(points = {{-165.9, 22}, {-160, 22}, {-160, 12}, {-178, 12}, {-178, 6}}, color = {0, 0, 255}));
    connect(wheels_differential.flange_b, wheelBearing.flange_a)
      annotation (Line(points={{40,26},{46,26}}, color={0,0,0}));
    connect(wheelBearing.flange_b, wheel.flangeR)
      annotation (Line(points={{66,26},{72,26}}, color={0,0,0}));
    connect(lossyGear.flange_b, lossyDifferential.flange_a) annotation (
      Line(points = {{-12, 26}, {-6, 26}}, color = {0, 0, 0}));
    connect(wheels_differential.flange_a, lossyDifferential.flange_b) annotation (
      Line(points = {{20, 26}, {14, 26}}, color = {0, 0, 0}));
    connect(eleDrive.flange_a, rotBearingFric.flange_a)
      annotation (Line(points={{-98,26},{-92,26}}, color={0,0,0}));
    connect(rotBearingFric.flange_b, mP0.flange_a) annotation (
      Line(points = {{-72, 26}, {-64, 26}}, color = {0, 0, 0}));
    connect(mP0.flange_b, lossyGear.flange_a) annotation (
      Line(points = {{-44, 26}, {-32, 26}}, color = {0, 0, 0}));
    connect(eleDrive.tauRef, propDriver.tauRef) annotation (Line(points={{
            -119.4,26},{-126,26},{-126,-4},{-133,-4}}, color={0,0,127}));
    connect(propDriver.V, speedSensor.v) annotation (Line(points={{-144,-15.2},
            {-146,-15.2},{-146,-38},{150,-38},{150,-9}}, color={0,0,127}));
    annotation (
      Diagram(coordinateSystem(extent = {{-200, -60}, {180, 80}})),
      experiment(StopTime = 1800, Interval = 0.760135, Tolerance = 1e-06, StartTime = 0),
      Icon(coordinateSystem(extent = {{-200, -60}, {180, 80}})));
  end Nissan;

  package Records
    record TeslaS60
      // AC Induction Machine
      parameter Modelica.Units.SI.Mass m = 2099+75;
      // Gross Vehicle Weight
      parameter Real gearRatioEff = 6/3.12;
      parameter Real gear1Ratio = 3.12;
      // Tesla S P90D
      parameter Real gear2Ratio = 6/3.12;
      parameter Real differentialRatio = 3.12;
      parameter Modelica.Units.SI.Length radius = 0.3525;
      // 245/35 R21
      parameter Modelica.Units.SI.MomentOfInertia J_rotor = 0.06;
      // Stima
      parameter Modelica.Units.SI.MomentOfInertia J_d = 0.07;
      parameter Modelica.Units.SI.MomentOfInertia J_w = 5.12;
      parameter Real Cx = 0.24;
      parameter Modelica.Units.SI.Area S = 2.34;
      // 25.2 square feet
      parameter Real fc = 0.013;
      parameter Modelica.Units.SI.Density rho = 1.225;
      parameter Real p = 2;
      parameter Modelica.Units.SI.Power MaxPower = 225e3;
      parameter Modelica.Units.SI.Torque MaxTorque = 430;
      parameter Modelica.Units.SI.AngularVelocity MaxOmega = 1571;
      // Tesla S P90D
      parameter Modelica.Units.SI.Voltage motorVoltage = 320;
      parameter Modelica.Units.SI.Voltage DCVoltage = 366;
      parameter Modelica.Units.SI.ElectricCharge QCellNom = 3.2*3600;
      parameter Modelica.Units.SI.Voltage ECellMin = 2.5;
      parameter Modelica.Units.SI.Voltage ECellMax = 4.2;
      parameter Modelica.Units.SI.Current ICellMax = 750;
      parameter Integer ns = 6*16;
      parameter Integer np = 52;
      parameter Real kContr(unit = "N.m/(m/s)") = 2000;
      annotation (
        Icon(graphics={  Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 57}, extent = {{-99, 19}, {99, -19}}, textString = "TeslaS60"), Text(origin = {0, -33}, textColor = {0, 0, 255}, extent = {{-100, 15}, {100, -15}}, textString = "%name")}));
    end TeslaS60;

    record NissanLEAF
      // SMPM Motor
      parameter Modelica.Units.SI.Mass m = 1643 +75;
      // Gross Vehicle Weight
      parameter Real gearRatioEff = 5.5/(74/17);
      parameter Real gear1Ratio = 31/17;
      parameter Real gear2Ratio = 5/(74/17);
      parameter Real differentialRatio = 74/17;
      parameter Modelica.Units.SI.Length radius = 0.316;
      parameter Modelica.Units.SI.MomentOfInertia J_rotor = 0.0347;
      parameter Modelica.Units.SI.MomentOfInertia J_d = 0.07;
      parameter Modelica.Units.SI.MomentOfInertia J_w = 3.22;
      parameter Real Cx = 0.29;
      parameter Modelica.Units.SI.Area S = 2.276;
      parameter Real fc = 0.013;
      parameter Modelica.Units.SI.Density rho = 1.225;
      parameter Real p = 2;
      parameter Modelica.Units.SI.Power MaxPower = 80e3;
      parameter Modelica.Units.SI.Torque MaxTorque = 280;
      parameter Modelica.Units.SI.AngularVelocity MaxOmega = 1088;
      parameter Modelica.Units.SI.AngularVelocity BaseOmega = 0.2649*1088;
      parameter Modelica.Units.SI.Voltage motorVoltage = 300;
      // EM 61
      parameter Modelica.Units.SI.Voltage DCVoltage = 360;
      parameter Modelica.Units.SI.ElectricCharge QCellNom = 33.1*3600;
      parameter Modelica.Units.SI.Voltage ECellMin = 2.5;
      parameter Modelica.Units.SI.Voltage ECellMax = 4.2;
      parameter Modelica.Units.SI.Current ICellMax = 520;
      parameter Integer ns = 2*48;
      parameter Integer np = 2;
      parameter Real kContr(unit = "N.m/(m/s)") = 2000;
      annotation (
        Icon(graphics={  Rectangle(lineColor = {0, 170, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 59}, extent = {{-78, 21}, {78, -21}},
              textString="Nissan LEAF",
              textColor={0,0,0}),                                                                                                                                                                                                        Text(origin = {-1, -33}, textColor = {0, 0, 255}, extent = {{-43, 17}, {43, -17}}, textString = "%name")}));
    end NissanLEAF;
  end Records;
  annotation (
    uses(Modelica(version = "4.0.0"), EHPTlib(version = "2.1.4")));
end BevModels;

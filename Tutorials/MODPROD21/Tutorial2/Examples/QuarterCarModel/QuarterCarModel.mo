within ;
package QuarterCarModel
  model QuarterCarModel
    Modelica.Mechanics.Translational.Components.Mass wheel(L=0.0, m=40, s(fixed=true, start=0.0), v(fixed=true, start=0.0)) annotation (
      Placement(transformation(extent={{10,-10},{30,10}})));
    Modelica.Mechanics.Translational.Components.Mass chassis(L=0.0, m=400, s(fixed=true, start=0.0), v(fixed=true, start=0.0)) annotation (
      Placement(transformation(extent={{70,-10},{90,10}})));
    Modelica.Mechanics.Translational.Components.SpringDamper springDamper_chassis(s_rel0=0.0, c=15000, d=1000) annotation (
      Placement(transformation(extent={{40,-10},{60,10}})));
    Modelica.Mechanics.Translational.Components.Spring spring_wheel(c=150000, s_rel0=0.0) annotation (
      Placement(transformation(extent={{-20,-10},{0,10}})));
    Modelica.Mechanics.Translational.Sources.Move ground annotation (
      Placement(transformation(extent={{-50,-10},{-30,10}})));
    Modelica.Blocks.Interfaces.RealInput road annotation (
      Placement(transformation(extent={{-120,-10},{-100,10}})));
  protected
    Modelica.Blocks.Sources.Constant zero annotation (
      Placement(visible = true, transformation(extent = {{-90, -30}, {-70, -10}}, rotation = 0)));
  equation
    connect(zero.y, ground.u[3]) annotation (
      Line(points = {{-69, -20}, {-69, -20}, {-60, -20}, {-60, 0}, {-52, 0}, {-52, 1.33333}}, color = {0, 0, 127}));
    connect(zero.y, ground.u[2]) annotation (
      Line(points = {{-69, -20}, {-60, -20}, {-60, 0}, {-52, 0}}, color = {0, 0, 127}));
    connect(springDamper_chassis.flange_b, chassis.flange_a) annotation (
      Line(points = {{60, 0}, {60, 0}, {70, 0}}, color = {0, 127, 0}));
    connect(wheel.flange_b, springDamper_chassis.flange_a) annotation (
      Line(points = {{30, 0}, {30, 0}, {40, 0}}, color = {0, 127, 0}));
    connect(spring_wheel.flange_b, wheel.flange_a) annotation (
      Line(points = {{0, 0}, {4, 0}, {10, 0}}, color = {0, 127, 0}));
    connect(ground.flange, spring_wheel.flange_a) annotation (
      Line(points = {{-30, 0}, {-25, 0}, {-20, 0}}, color = {0, 127, 0}));
    connect(road, ground.u[1]) annotation (
      Line(points = {{-110, 0}, {-52, 0}, {-52, -1.33333}}, color = {0, 0, 127}));
    annotation (
      Diagram(coordinateSystem(extent={{-100,-40},{100,40}})), Icon(coordinateSystem(extent={{-100,-40},{100,40}})));
  end QuarterCarModel;

  model QuarterCarModelEq
    Real eta_c(fixed=true, start=0.0);
    Real der_eta_c(fixed=true, start=0.0);
    Real eta_w(fixed=true, start=0.0);
    Real der_eta_w(fixed=true, start=0.0);
    Modelica.Blocks.Interfaces.RealInput road annotation (
      Placement(transformation(extent={{-120,-10},{-100,10}})));
    parameter Real m_w = 40;
    parameter Real m_c = 400;
    parameter Real k_w = 150000;
    parameter Real k_c = 15000;
    parameter Real d_c = 1000;
  equation
    der(eta_c) = der_eta_c;
    der(eta_w) = der_eta_w;
    m_c*der(der_eta_c) = k_c*(eta_w-eta_c) + d_c*(der_eta_w-der_eta_c);
    m_w*der(der_eta_w) = k_w*(road - eta_w) - k_c*(eta_w-eta_c) - d_c*(der_eta_w-der_eta_c);
    annotation (
      Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end QuarterCarModelEq;

  model ReferenceSystem
    Modelica.Blocks.Sources.Step Road(height = 0.1, startTime = 1.0)  annotation (
      Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    QuarterCarModelEq quarterCarModel
      annotation (Placement(visible = true, transformation(extent = {{10, -4}, {30, 4}}, rotation = 0)));
  equation
    connect(Road.y, quarterCarModel.road)
      annotation (Line(points={{-9,0}, {9, 0}}, color={0,0,127}));
  end ReferenceSystem;

  package DisplacementDisplacement
    model Chassis
      parameter Real m_c = 400;
      parameter Real k_c = 15000;
      parameter Real d_c = 1000;
      Modelica.Blocks.Interfaces.RealInput eta_w annotation (
        Placement(transformation(extent={{-120,20},{-100,40}})));
      Modelica.Blocks.Interfaces.RealInput der_eta_w annotation (
        Placement(transformation(extent={{-120,-40},{-100,-20}})));
      Modelica.Blocks.Interfaces.RealOutput eta_c(fixed=true, start=0.0) annotation (
        Placement(transformation(extent={{100,20},{120,40}})));
      Modelica.Blocks.Interfaces.RealOutput der_eta_c(fixed=true, start=0.0) annotation (
        Placement(transformation(extent={{100,-40},{120,-20}})));
    equation
      der(eta_c) = der_eta_c;
      m_c*der(der_eta_c) = k_c*(eta_w-eta_c) + d_c*(der_eta_w-der_eta_c);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Chassis;

    model Wheel
      parameter Real m_w = 40;
      parameter Real k_w = 150000;
      parameter Real k_c = 15000;
      parameter Real d_c = 1000;

      Modelica.Blocks.Interfaces.RealInput eta_c annotation (
        Placement(transformation(extent={{-120,20},{-100,40}})));
      Modelica.Blocks.Interfaces.RealInput der_eta_c annotation (
        Placement(transformation(extent={{-120,-40},{-100,-20}})));
      Modelica.Blocks.Interfaces.RealOutput eta_w(fixed=true, start=0.0) annotation (
        Placement(transformation(extent={{100,20},{120,40}})));
      Modelica.Blocks.Interfaces.RealOutput der_eta_w(fixed=true, start=0.0) annotation (
        Placement(transformation(extent={{100,-40},{120,-20}})));
      Modelica.Blocks.Interfaces.RealInput road annotation (
        Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-110})));
    equation
      der(eta_w) = der_eta_w;
      m_w*der(der_eta_w) = k_w*(road - eta_w) - k_c*(eta_w-eta_c) - d_c*(der_eta_w-der_eta_c);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Wheel;

    model CompositeModel
      Chassis chassis
        annotation (
          Placement(visible = true, transformation(extent = {{10, -10}, {30, 10}}, rotation = 0)));
      Wheel wheel
        annotation (
          Placement(visible = true, transformation(extent = {{-30, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step(height=0.1, startTime=1)
        annotation (
          Placement(visible = true, transformation(extent = {{-50, -50}, {-30, -30}}, rotation = 0)));
    equation
      connect(step.y, wheel.road) annotation (
        Line(points = {{-29, -40}, {-20, -40}, {-20, -11}}, color = {0, 0, 127}));
      connect(chassis.der_eta_c, wheel.der_eta_c) annotation (
        Line(points = {{31, -3}, {40, -3}, {40, -20}, {-40, -20}, {-40, -3}, {-31, -3}}, color = {0, 0, 127}));
      connect(chassis.eta_c, wheel.eta_c) annotation (
        Line(points = {{31, 3}, {40, 3}, {40, 20}, {-40, 20}, {-40, 3}, {-31, 3}}, color = {0, 0, 127}));
      connect(wheel.der_eta_w, chassis.der_eta_w) annotation (
        Line(points = {{-9, -3}, {9, -3}}, color = {0, 0, 127}));
      connect(wheel.eta_w, chassis.eta_w) annotation (
        Line(points = {{-9, 3}, {9, 3}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end CompositeModel;
  end DisplacementDisplacement;

  package DisplacementForce
    model Chassis
      parameter Real m_w = 40;
      parameter Real m_c = 400;
      parameter Real k_w = 150000;
      parameter Real k_c = 15000;
      parameter Real d_c = 1000;

      Modelica.Blocks.Interfaces.RealInput F annotation (
        Placement(transformation(extent={{-120,-10},{-100,10}})));
      Modelica.Blocks.Interfaces.RealOutput eta_c(fixed=true, start=0.0) annotation (
        Placement(transformation(extent={{100,20},{120,40}})));
      Modelica.Blocks.Interfaces.RealOutput der_eta_c(fixed=true, start=0.0) annotation (
        Placement(transformation(extent={{100,-40},{120,-20}})));
      Modelica.Blocks.Interfaces.RealInput road annotation (
        Placement(transformation(extent={{-10,-10},{10,10}},rotation=90,origin={0,-110})));

      Real eta_w(fixed=true, start=0.0);
      Real der_eta_w(fixed=true, start=0.0);
    equation
      der(eta_c) = der_eta_c;
      der(eta_w) = der_eta_w;
      m_c*der(der_eta_c) = k_c*(eta_w-eta_c) + d_c*(der_eta_w-der_eta_c);
      m_w*der(der_eta_w) = k_w*(road - eta_w) - F;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Chassis;

    model Wheel
      parameter Real m_w = 40;
      parameter Real k_w = 150000;
      parameter Real k_c = 15000;
      parameter Real d_c = 1000;

      Modelica.Blocks.Interfaces.RealInput eta_c annotation (
        Placement(transformation(extent={{-120,20},{-100,40}})));
      Modelica.Blocks.Interfaces.RealInput der_eta_c annotation (
        Placement(transformation(extent={{-120,-40},{-100,-20}})));
      Modelica.Blocks.Interfaces.RealOutput F annotation (
        Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealInput road annotation (
        Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-110})));

      Real eta_w(fixed=true, start=0.0);
      Real der_eta_w(fixed=true, start=0.0);
    equation
      der(eta_w) = der_eta_w;
      m_w*der(der_eta_w) = k_w*(road - eta_w) - F;
      F = k_c*(eta_w-eta_c) + d_c*(der_eta_w-der_eta_c);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Wheel;

    model CompositeModel
      Chassis chassis annotation (
        Placement(visible = true, transformation(extent = {{10, -10}, {30, 10}}, rotation = 0)));
      Wheel wheel annotation (
        Placement(visible = true, transformation(extent = {{-30, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step(height=0.1, startTime=1) annotation (
        Placement(visible = true, transformation(extent = {{-50, -50}, {-30, -30}}, rotation = 0)));
    equation
      connect(step.y, chassis.road) annotation(
        Line(points = {{-29, -40}, {20, -40}, {20, -11}}, color = {0, 0, 127}));
      connect(step.y, wheel.road) annotation(
        Line(points = {{-29, -40}, {-20, -40}, {-20, -11}}, color = {0, 0, 127}));
      connect(chassis.eta_c, wheel.eta_c) annotation(
        Line(points = {{31, 3}, {40, 3}, {40, 20}, {-40, 20}, {-40, 3}, {-31, 3}}, color = {0, 0, 127}));
      connect(chassis.der_eta_c, wheel.der_eta_c) annotation(
        Line(points = {{31, -3}, {40, -3}, {40, -20}, {-40, -20}, {-40, -3}, {-31, -3}}, color = {0, 0, 127}));
      connect(wheel.F, chassis.F) annotation(
        Line(points = {{-9, 0}, {9, 0}}, color = {0, 0, 127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end CompositeModel;
  end DisplacementForce;
  annotation(uses(Modelica(version="3.2.3")));
end QuarterCarModel;
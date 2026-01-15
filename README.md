flowchart TB
    A[Autonomous Vehicle]

    A --> PM[1.0 Project Management]
    A --> ME[2.0 Mechanical]

    %% Project Management Branch
    PM --> PM1[1.1 Problem formulation]
    PM --> PM2[1.2 Identify goals and skills]
    PM --> PM3[1.3 Scheduling]
    PM --> PM4[1.4 Budgeting]
    PM --> PM5[1.5 Report Writing]
    PM --> PM6[1.6 Conceptual Design]

    %% Mechanical → Hull Design
    ME --> HD[2.1 Hull Design]
      HD --> HD1[2.1.1 Calculations (Buoyancy & Sizing)]
      HD --> HD2[2.1.2 Material selection]
      HD --> HD3[2.1.3 Drawings]
      HD --> HD4[2.1.4 Construction]
      HD --> HD5[2.1.5 Testing]

    %% Mechanical → Propulsion
    ME --> PR[2.2 Propulsion]
      PR --> PR1[2.2.1 Thrust calculation]
      PR --> PR2[2.2.2 Motor calculation]
      PR --> PR3[2.2.3 Hull design]
      PR --> PR4[2.2.4 Electrical]
      PR --> PR5[2.2.5 Testing]

    %% Mechanical → Steering
    ME --> ST[2.3 Steering]
      ST --> ST1[2.3.1 Motor selection]
      ST --> ST2[2.3.2 Component integration]
      ST --> ST3[2.3.3 Hull design]
      ST --> ST4[2.3.4 Electrical]
      ST --> ST5[2.3.5 Testing]

    %% Mechanical → Waterproofing
    ME --> WP[2.4 Waterproofing]
      WP --> WP1[2.4.1 Design]
      WP --> WP2[2.4.2 Testing]

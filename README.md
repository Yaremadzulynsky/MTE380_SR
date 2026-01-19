
# GROUP CONTRACT 

**Project:** Search & Rescue
**Course:** MTE380  
**Term:** 3B  

---

## 1. Purpose
This contract defines mandatory expectations for participation, professionalism, and accountability. Its purpose is to ensure fair workload distribution, timely progress, and successful project completion. By signing, all members agree to be held to these standards.

---

## 2. Attendance & Punctuality
- All members must attend all scheduled meetings and arrive on time.  
- <5 minutes late: may join quietly; catching up is optional.  
- >5 minutes late without notice:
  - Must explain absence,
  - Accept responsibility for missed work,
- More than **two (3) unexcused absences** may result in dismissal from the group and instructor notification.

**Initials:** _______

---

## 3. Academic Integrity
- Any plagiarism or academic dishonesty will be reported immediately.
- All members agree to comply with university Academic Honesty policies.

**Initials:** _______

---

## 4. Professional Conduct
- Respect is mandatory. Personal attacks, dismissive language, or hostility are unacceptable.
- Criticism must address ideas, not individuals.
- Violations result in:
  1. Formal warning,
  2. Documentation,
  3. Escalation to the instructor.

**Initials:** _______

---

## 5. Leadership & Decision-Making

- The group designates **two Integration Leads**:
  - **Hardware Integration Lead (Shiv)**  – responsible for coordinating all hardware work, ensuring subsystem compatibility, verifying interfaces, organizing handoff points, and overseeing physical integration.
  - **Software Integration Lead (Yarema)** – responsible for coordinating all software work, managing code structure, merge schedules, API/interface consistency, and ensuring the final software stack integrates cleanly with hardware and other components.

- Together, these two Integration Leads function as the **Project Management Team**, responsible for:
  - Scheduling and coordinating timelines,
  - Running meetings and setting agendas,
  - Tracking responsibilities and deliverables,
  - Identifying blockers early and redistributing work if needed,
  - Ensuring hardware–software compatibility at all stages of the project.

- Decision-making follows this hierarchy:
  1. **Consensus** among group members (preferred),
  2. **Majority vote** if consensus cannot be reached,
  3. **Integration Leads jointly decide** if the issue is time-critical or impacts system-level compatibility,
     - If Hardware vs. Software decisions conflict, the Integration Leads must discuss and produce a unified system-level decision.

- The Integration Leads **are not responsible for compensating for underperforming members**. Their role is to coordinate—not to carry extra workload due to missed deadlines or incomplete work.

**Initials:** _______

---

## 6. Deadlines & Work Quality
- Missed deadlines without prior notice:
  - Must be completed within 24 hours,
  - Trigger one additional assigned task.
- Repeated offenses:
  - Second: formal warning,
  - Third: instructor notification and grading adjustment.
- All work must be complete, functional, and integration-ready. Low-effort work counts as a missed deadline.

**Initials:** _______

---

## 7. Contribution & Accountability
- Weekly progress updates are mandatory.
- **Three consecutive weeks of inadequate contribution** may result in:
  - Task reassignment,
  - Documentation,
  - Instructor escalation,
  - Peer evaluation penalties.

**Initials:** _______

---

## 8. Meetings & Focus
- No interrupting or cross-talking.
- Disengagement or disruption results in:
  1. Reminder,
  2. Private warning,
  3. Documentation and escalation.

**Initials:** _______

---
## 9. Documentation & Report Writing Responsibilities

- All members are responsible for writing **their assigned sections** of the report at a professional standard appropriate for an engineering course.

- Each section must include:
  - Clear, concise technical writing  
  - Figures/tables with proper captions  
  - References where appropriate  
  - Consistent formatting following group standards  

- **Copy-pasting raw notes, unedited text, or last-minute drafts is unacceptable** and will be treated as a missed deadline.

- The report will follow a clear structure defined by the Integration Leads.  
  All section headings, figure styles, notation, and file naming **must follow the group’s unified template**.

- The group designates a **Documentation Lead (Seb)** responsible for:
  - Unifying tone and formatting  
  - Ensuring sections do not contradict each other  
  - Checking that the document matches the system architecture and actual implementation  
  - Making the final assembled version “submission-ready”  

- The Documentation Lead **does not rewrite poor-quality work**.  
  Any section returned for revision must be fixed within 24 hours by the original author.

- Repeat submission of incomplete or low-effort sections will trigger:
  1. A formal warning  
  2. Documentation of the incident  
  3. Instructor escalation if unresolved  

**Initials:** _______


## 10. Agreement
By signing below, each member agrees to abide by this contract and accepts its enforcement.

**Date:** ______________________

| Print Name | Signature |
|-----------|-----------|
| | |
| | |
| | |
| | |








```mermaid
gantt
    title MTE 380 Search & Rescue Robot – 3 Month Plan (Numbered WBS)
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section 1.0 Project Management & Systems
    1.1 Requirements & Constraints              :done,    1_1, 2026-01-15, 2026-01-22
    1.2 System Architecture                     :active,  1_2, 2026-01-20, 2026-01-30
    1.3 Interface Definitions                   :         1_3, 2026-01-25, 2026-02-05
    1.4 System State Machine                    :         1_4, 2026-01-28, 2026-02-10
    1.5 Integration Planning                    :         1_5, 2026-02-03, 2026-02-18
    1.6 Scheduling & Milestones                 :         1_6, 2026-01-15, 2026-02-12

    section 2.0 Mechanical
    2.1 Chassis Design                          :done,    2_1, 2026-01-15, 2026-01-27
    2.2 Drive System                            :active,  2_2, 2026-01-20, 2026-02-02
    2.3 Gripper & Capture Mechanism             :         2_3, 2026-01-25, 2026-02-07
    2.4 Mechanical Assembly                     :         2_4, 2026-02-01, 2026-02-14
    2.5 Mechanical Testing                      :         2_5, 2026-02-10, 2026-03-05

    section 3.0 Electrical
    3.1 Power System                            :         3_1, 2026-01-20, 2026-02-01
    3.2 Motor Drivers                           :         3_2, 2026-01-25, 2026-02-10
    3.3 Sensors                                 :         3_3, 2026-02-01, 2026-02-12
    3.4 Wiring & Integration                    :         3_4, 2026-02-05, 2026-02-18
    3.5 Electrical Testing                      :         3_5, 2026-02-12, 2026-03-08

    section 4.0 Software
    4.1 Systems & Behavior Software             :         4_1, 2026-01-25, 2026-03-08
    4.1.1 State Machine                         :         4_1_1, 2026-02-10, 2026-03-01
    4.1.2 Navigation Logic                       :         4_1_2, 2026-02-15, 2026-03-05
    4.1.3 High-Level Decisions                  :         4_1_3, 2026-02-20, 2026-03-08
    4.1.4 Error Handling / Recovery             :         4_1_4, 2026-02-25, 2026-03-12

    4.2 Computer Vision (Perception)            :         4_2, 2026-02-01, 2026-03-05
    4.2.1 Detect Lego Figure                    :         4_2_1, 2026-02-05, 2026-02-18
    4.2.2 Detect Path / Colors                  :         4_2_2, 2026-02-10, 2026-02-22
    4.2.3 Detect Danger Zone                    :         4_2_3, 2026-02-15, 2026-02-28
    4.2.4 Provide Coordinates / Features        :         4_2_4, 2026-02-18, 2026-03-05

    4.3 Controls & Motion                       :         4_3, 2026-02-01, 2026-03-10
    4.3.1 Line Following Controller             :         4_3_1, 2026-02-08, 2026-02-21
    4.3.2 Drive Commands (Velocity / Turn)      :         4_3_2, 2026-02-01, 2026-02-12
    4.3.3 Gripper Control                       :         4_3_3, 2026-02-10, 2026-02-22
    4.3.4 Movement Primitives (Forward/Turn)    :         4_3_4, 2026-02-12, 2026-02-25

    4.4 Software Testing                        :         4_4, 2026-02-15, 2026-04-10
    4.4.1 Unit Tests                            :         4_4_1, 2026-02-15, 2026-03-10
    4.4.2 Integration Tests                     :         4_4_2, 2026-03-01, 2026-04-10

    section 5.0 Testing & Validation
    5.1 Unit Testing                            :         5_1, 2026-03-10, 2026-03-20
    5.2 Subsystem Integration                   :         5_2, 2026-03-12, 2026-03-25
    5.3 Full System Testing                     :         5_3, 2026-03-20, 2026-04-08
    5.4 Game-Day Rehearsals                     :crit,    5_4, 2026-04-05, 2026-04-12

    section 6.0 Documentation & Reporting
    6.1 Design Rationale                        :         6_1, 2026-02-10, 2026-03-10
    6.2 System Architecture Description         :         6_2, 2026-02-15, 2026-03-12
    6.3 Mechanical Design Section               :         6_3, 2026-03-01, 2026-03-20
    6.4 Electrical Design Section               :         6_4, 2026-03-01, 2026-03-20
    6.5 Software Design Section                 :         6_5, 2026-03-10, 2026-03-25
    6.6 Testing & Results                       :         6_6, 2026-03-20, 2026-04-12
    6.7 Lessons Learned                         :         6_7, 2026-04-05, 2026-04-12

```

```mermaid
    flowchart TB
        PM["1.0 Project Management & Systems"]
        PM --> PM1["1.1 Requirements & Constraints"]
        PM --> PM2["1.2 System Architecture"]
        PM --> PM3["1.3 Interface Definitions"]
        PM --> PM4["1.4 System State Machine"]
        PM --> PM5["1.5 Integration Planning"]
        PM --> PM6["1.6 Scheduling & Milestones"]
```

```mermaid
    flowchart TB
        ME["2.0 Mechanical"]
        ME --> ME1["2.1 Chassis Design"]
        ME --> ME2["2.2 Drive System"]
        ME --> ME3["2.3 Gripper & Capture Mechanism"]
        ME --> ME4["2.4 Mechanical Assembly"]
        ME --> ME5["2.5 Mechanical Testing"]

```

```mermaid
    flowchart TB
        EE["3.0 Electrical"]
        EE --> EE1["3.1 Power System"]
        EE --> EE2["3.2 Motor Drivers"]
        EE --> EE3["3.3 Sensors"]
        EE --> EE4["3.4 Wiring & Integration"]
        EE --> EE5["3.5 Electrical Testing"]

```
```mermaid
flowchart TB
    SW["4.0 Software"]

    %% HIGH LEVEL
    SW --> SYS["4.1 Systems & Behavior Software"]
    SW --> CV["4.2 Computer Vision (Perception)"]
    SW --> CTRL["4.3 Controls & Motion"]
    SW --> TEST["4.4 Software Testing"]

    %% SYSTEMS LAYER (Decision-making)
    SYS --> SYS1["4.1.1 State Machine"]
    SYS --> SYS2["4.1.2 Navigation Logic"]
    SYS --> SYS3["4.1.3 High-Level Decisions"]
    SYS --> SYS4["4.1.4 Error Handling / Recovery"]

    %% CV LAYER (Perception only)
    CV --> CV1["4.2.1 Detect Lego Figure"]
    CV --> CV2["4.2.2 Detect Path / Colors"]
    CV --> CV3["4.2.3 Detect Danger Zone"]
    CV --> CV4["4.2.4 Provide Coordinates / Features"]
    
    %% CONTROLS LAYER (Low-level actuation)
    CTRL --> CTRL1["4.3.1 Line Following Controller"]
    CTRL --> CTRL2["4.3.2 Drive Commands (Velocity / Turn)"]
    CTRL --> CTRL3["4.3.3 Gripper Control"]
    CTRL --> CTRL4["4.3.4 Movement Primitives (Forward/Turn)"]

    %% TESTING
    TEST --> TEST1["4.4.1 Unit Tests"]
    TEST --> TEST2["4.4.2 Integration Tests"]
```

```mermaid

    flowchart TB
        TS["5.0 Testing & Validation"]
    
        TS --> TS1["5.1 Unit Testing"]
        TS --> TS2["5.2 Subsystem Integration"]
        TS --> TS3["5.3 Full System Testing"]
        TS --> TS4["5.4 Game-Day Rehearsals"]

```
```mermaid
    flowchart TB
        RP["6.0 Documentation & Reporting"]
    
        RP --> RP1["6.1 Design Rationale"]
        RP --> RP2["6.2 System Architecture Description"]
        RP --> RP3["6.3 Mechanical Design Section"]
        RP --> RP4["6.4 Electrical Design Section"]
        RP --> RP5["6.5 Software Design Section"]
        RP --> RP6["6.6 Testing & Results"]
        RP --> RP7["6.7 Lessons Learned"]

```





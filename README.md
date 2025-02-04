# **Autonomous Taxiing of Aircraft: Controller Comparison and Sensor Fusion in Detail**

### **[Project Repository](https://anujithm.github.io/Autonomous-Taxiing-of-Aircraft.github.io/)**

---

## **1. Introduction**

This research focuses on developing an autonomous taxiing system for aircraft, aiming to enhance safety, efficiency, and precision in ground operations. 
The project includes the implementation of control strategies for smooth navigation and the integration of robust sensor fusion techniques to ensure reliable obstacle detection and avoidance, which will be discussed in detail below.


---

## **2. Key Contributions**

1. **Controller Comparison:**
   - Implementation and evaluation of multiple controllers:
     - **Proportional-Derivative (PD) Controller**
     - **Sliding Mode Control (SMC)**
     - **Linear Quadratic Regulator (LQR)**
     - **Stanley Controller**
   - Comprehensive analysis of controller performance under varying conditions, including sharp turns, intersections, and obstacle-laden environments.

2. **Sensor Fusion:**
   - Integration of ultrasonic sensor data with an object detection model to enhance obstacle detection accuracy.
   - The fusion technique allows for redundancy in obstacle recognition, ensuring higher reliability even in challenging scenarios.

3. **Real-Time State Management:**
   - Development of automatic and manual switching mechanisms between control modes for flexible operation during different taxiing scenarios.

---

## **3. System Architecture**

The system comprises three key modules:
- **Control Module:** Handles the dynamic adjustment of aircraft steering and speed based on real-time sensor inputs.
- **Sensor Fusion Module:** Merges data from ultrasonic sensors and vision-based object detection to improve the accuracy of obstacle detection.
- **State Management Module:** Manages transitions between manual, automatic, and emergency stop states.

---

## **4. Flowchart of the Algorithm**

### **Flowchart**

\`\`\`mermaid
flowchart TD
    A[Start] --> B{Read current state}
    B --> |State = 0 (Manual)| C[Manual Control Mode]
    B --> |State = 1 (Automatic)| D[Automatic Control Mode]
    B --> |State = 2 (Stop)| E[Emergency Stop Mode]
    C --> F{Read user control input}
    F --> G[Publish velocity commands]
    G --> B
    D --> H{Read lane detection and object data}
    H --> |Data = None| I[Stop vehicle and switch to Manual Mode]
    H --> |Object detected| J{Check ultrasonic distance}
    J --> |Distance < 25 cm| I
    J --> |Distance ≥ 25 cm| K[Continue Automatic Mode]
    H --> |Lane data valid| L{Controller Selection}
    L --> |PD Controller| M[Apply PD control law]
    L --> |SMC| N[Apply SMC control law]
    L --> |LQR| O[Apply LQR control law]
    L --> |Stanley| P[Apply Stanley control law]
    M --> Q[Publish velocity commands]
    N --> Q
    O --> Q
    P --> Q
    Q --> B
    E --> R[Set velocity to zero]
    R --> B
\`\`\`

---

## **5. Controller Comparison**

### **5.1 Proportional-Derivative (PD) Controller**
- Simple yet effective controller for basic lane following tasks.
- Provides quick responsiveness to changes in trajectory.

### **5.2 Sliding Mode Control (SMC)**
- Robust against external disturbances and model uncertainties.
- Ensures stable control even under dynamic environmental changes.

### **5.3 Linear Quadratic Regulator (LQR)**
- Optimal control strategy minimizing a cost function that balances control effort and system error.
- Effective in achieving smooth trajectory tracking with minimal control effort.

### **5.4 Stanley Controller**
- Primarily designed for path tracking in autonomous vehicles.
- Integrates both cross-track error and heading error to generate steering commands.

**Performance Metrics:**
- Tracking accuracy
- Response time
- Stability under varying conditions

---

## **6. Sensor Fusion Approach**

The fusion of ultrasonic sensor data with the object detection model enhances the reliability of obstacle detection. The ultrasonic sensor acts as a fail-safe mechanism when visual data is unreliable, such as in low-light conditions or due to occlusions.

### **Sensor Fusion Workflow:**
1. Object detection model identifies potential obstacles.
2. Ultrasonic sensor measures distance to detected objects.
3. If the distance is less than a safety threshold (25 cm), the system triggers an emergency stop.

---

## **7. Experimental Results**

The system was tested under various scenarios, including sharp turns, intersections, and obstacle-dense environments. Key findings include:
- **Stanley Controller** performed exceptionally well in sharp turns.
- **LQR Controller** provided smoother trajectories with less oscillation.
- **PD Controller** offered quick responsiveness but struggled with stability at high speeds.
- **SMC** showed robustness against disturbances but required fine-tuning for optimal performance.

---

## **8. Conclusion**

This research demonstrates a robust approach to autonomous aircraft taxiing by integrating advanced control strategies with sensor fusion techniques. The comparative study of controllers provides insights into their strengths and limitations, guiding future improvements in autonomous ground navigation systems.

For more details, refer to the [Project Repository](https://anujithm.github.io/Autonomous-Taxiing-of-Aircraft.github.io/).


## Acknowledgments
Parts of this project page were adopted from the [Nerfies](https://nerfies.github.io/) page.

## Website License
<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.

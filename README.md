### Optimierung und Simulation von Design und Leistung der Schleifspindel (Optimizing and simulating design & performance of grinding spindle)
The system simulates spindle performance under various conditions, evaluates metrics like vibration, temperature, and bearing life, and uses a k-Nearest Neighbors (k-NN) algorithm for maintenance prediction utilizes Multi-Objective Genetic Algorithm (MOGA) to optimize spindle parameters to minimize vibration, maximize bearing life, and minimize temperature rise.

* Covers multiple aspects of spindle performance (power, vibration, temperature, wear, fatigue).
* Supports different scenarios and time-based analysis, making it versatile for various use cases.
* Encapsulates spindle configuration with setters and getters, ensuring clean data management.
* Checks parameter validity, returning error messages for out-of-range values.
* Runs simulations across three scenarios (High-Speed, High-Torque, Balanced) with different speed and load factors. Outputs detailed metrics like power, vibration, and bearing life.
* Simulates performance over a user-specified duration, tracking vibration, temperature, and load at 0.1-second intervals.
* Uses kNN on historical data to predict if maintenance is needed based on vibration, temperature, and other metrics.
* Uses Multi-Objective Genetic Algorithm (MOGA) to find Pareto-optimal spindle configurations, balancing vibration, bearing life, and temperature.
* Performance calculations :—
  |                         |                                     |
  |-------------------------|-------------------------------------|
  | Power Calculation        | Estimates required power based on wheel diameter and speed|
  | Vibration | Models vibration based on bearing type, speed, alignment, and tool interface|
  | Temperature Rise | Accounts for cooling type, speed, preload, and load|
  | Bearing Life (L10) | Calculates bearing life using dynamic load profiles and lubrication adjustments|
  | Spindle Fatigue Life | Estimates remaining life based on load-induced stress and S-N curve parameters|
  | Wheel Wear | Models wear based on load, peripheral speed, and duration, impacting vibration|
* Optimization using Multi-Objective Genetic Algorithm (MOGA) :—
  * Minimize vibration, maximize bearing life (negated for minimization), and minimize temperature rise.
  * Evolves a population of spindle configurations over generations.
  * Ranks solutions based on Pareto dominance, using crowding distance for diversity.
  * Uses BLX-α crossover for continuous parameters and uniform crossover for categorical ones, with polynomial mutation for continuous parameters.
  * Returns a Pareto front of optimal configurations with detailed parameters and objectives.

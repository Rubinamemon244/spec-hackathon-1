import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CognitivePlanningComponent.module.css';

/**
 * Cognitive Planning Component for VLA module
 * Visualizes how LLMs translate natural language to action sequences
 */
const CognitivePlanningComponent = ({ title = "Cognitive Planning Visualizer", description = "See how natural language commands are translated to robot actions" }) => {
  const [command, setCommand] = useState('');
  const [plan, setPlan] = useState(null);
  const [isProcessing, setIsProcessing] = useState(false);

  // Sample planning data
  const samplePlans = {
    "go to the kitchen and bring me a cup": [
      { id: 1, action: "NAVIGATE_TO_LOCATION", parameters: { location: "kitchen" }, description: "Navigate to kitchen" },
      { id: 2, action: "DETECT_OBJECT", parameters: { object_type: "cup" }, description: "Look for cup" },
      { id: 3, action: "GRASP_OBJECT", parameters: { object_type: "cup" }, description: "Pick up the cup" },
      { id: 4, action: "NAVIGATE_TO_LOCATION", parameters: { location: "user" }, description: "Return to user" },
      { id: 5, action: "PLACE_OBJECT", parameters: { object_type: "cup", location: "user" }, description: "Place cup for user" }
    ],
    "find the person in red and follow them": [
      { id: 1, action: "DETECT_PERSON", parameters: { color: "red" }, description: "Find person in red" },
      { id: 2, action: "FOLLOW_PERSON", parameters: { person_id: "target" }, description: "Start following" },
      { id: 3, action: "MONITOR_DISTANCE", parameters: { min_distance: 1.0 }, description: "Maintain safe distance" }
    ],
    "move forward two meters": [
      { id: 1, action: "NAVIGATE_TO_LOCATION", parameters: { x: 2.0, y: 0.0, z: 0.0 }, description: "Move forward 2 meters" },
      { id: 2, action: "VERIFY_POSITION", parameters: {}, description: "Confirm position" }
    ]
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (!command.trim()) return;

    setIsProcessing(true);

    // Simulate processing delay
    setTimeout(() => {
      const normalizedCommand = command.toLowerCase().trim();
      let foundPlan = null;

      // Check for exact matches first
      for (const [sampleCommand, planSteps] of Object.entries(samplePlans)) {
        if (normalizedCommand.includes(sampleCommand) || sampleCommand.includes(normalizedCommand)) {
          foundPlan = planSteps;
          break;
        }
      }

      // If no exact match, try partial matches
      if (!foundPlan) {
        for (const [sampleCommand, planSteps] of Object.entries(samplePlans)) {
          if (normalizedCommand.includes('kitchen') && normalizedCommand.includes('cup')) {
            foundPlan = samplePlans["go to the kitchen and bring me a cup"];
            break;
          } else if (normalizedCommand.includes('follow') && normalizedCommand.includes('person')) {
            foundPlan = samplePlans["find the person in red and follow them"];
            break;
          } else if (normalizedCommand.includes('move') && normalizedCommand.includes('forward')) {
            foundPlan = samplePlans["move forward two meters"];
            break;
          }
        }
      }

      // If still no match, create a simple plan
      if (!foundPlan) {
        foundPlan = [
          { id: 1, action: "PROCESS_COMMAND", parameters: { input: command }, description: `Process command: "${command}"` },
          { id: 2, action: "SPEAK", parameters: { text: `I'm processing your command: ${command}` }, description: "Acknowledge command" }
        ];
      }

      setPlan(foundPlan);
      setIsProcessing(false);
    }, 1500);
  };

  const getActionColor = (action) => {
    switch (action) {
      case 'NAVIGATE_TO_LOCATION': return '#007bff';
      case 'GRASP_OBJECT': return '#28a745';
      case 'PLACE_OBJECT': return '#ffc107';
      case 'DETECT_OBJECT': return '#17a2b8';
      case 'SPEAK': return '#6f42c1';
      case 'WAIT': return '#6c757d';
      default: return '#fd7e14';
    }
  };

  return (
    <div className={clsx('container', styles.planningContainer)}>
      <div className={clsx('row', styles.planningRow)}>
        <div className={clsx('col col--12')}>
          <div className={clsx('padding-horiz--md', styles.planningInterface)}>
            <h3>{title}</h3>
            <p>{description}</p>

            <form onSubmit={handleSubmit} className={styles.commandForm}>
              <div className={styles.inputGroup}>
                <label htmlFor="commandInput">Enter Natural Language Command:</label>
                <input
                  id="commandInput"
                  type="text"
                  value={command}
                  onChange={(e) => setCommand(e.target.value)}
                  placeholder="e.g., Go to the kitchen and bring me a cup"
                  className={styles.commandInput}
                  disabled={isProcessing}
                />
                <button
                  type="submit"
                  className={clsx('button button--primary', styles.planButton)}
                  disabled={isProcessing || !command.trim()}
                >
                  {isProcessing ? 'Processing...' : 'Generate Plan'}
                </button>
              </div>
            </form>

            {isProcessing && (
              <div className={styles.processingIndicator}>
                <div className={styles.spinner}></div>
                <p>Processing with LLM... Translating natural language to action sequence</p>
              </div>
            )}

            {plan && (
              <div className={styles.planVisualization}>
                <h4>Generated Action Plan:</h4>
                <div className={styles.planSteps}>
                  {plan.map((step, index) => (
                    <div key={step.id} className={clsx(styles.planStep, styles[`step-${index % 4}`])}>
                      <div className={styles.stepHeader}>
                        <span className={styles.stepNumber}>Step {index + 1}</span>
                        <span
                          className={styles.actionType}
                          style={{ backgroundColor: getActionColor(step.action) }}
                        >
                          {step.action}
                        </span>
                      </div>
                      <div className={styles.stepContent}>
                        <p className={styles.stepDescription}>{step.description}</p>
                        <details className={styles.stepDetails}>
                          <summary>Parameters</summary>
                          <pre className={styles.parameters}>
                            {JSON.stringify(step.parameters, null, 2)}
                          </pre>
                        </details>
                      </div>
                    </div>
                  ))}
                </div>

                <div className={styles.planMetrics}>
                  <div className={styles.metric}>
                    <span className={styles.metricLabel}>Total Steps:</span>
                    <span className={styles.metricValue}>{plan.length}</span>
                  </div>
                  <div className={styles.metric}>
                    <span className={styles.metricLabel}>Estimated Time:</span>
                    <span className={styles.metricValue}>{plan.length * 2-5} seconds</span>
                  </div>
                  <div className={styles.metric}>
                    <span className={styles.metricLabel}>Confidence:</span>
                    <span className={styles.metricValue}>85-95%</span>
                  </div>
                </div>
              </div>
            )}

            <div className={styles.planningInfo}>
              <h4>How Cognitive Planning Works:</h4>
              <div className={styles.infoGrid}>
                <div className={styles.infoCard}>
                  <h5>1. Language Understanding</h5>
                  <p>LLMs parse natural language to extract intent, objects, and spatial relationships</p>
                </div>
                <div className={styles.infoCard}>
                  <h5>2. Context Integration</h5>
                  <p>System considers robot state, environment, and constraints during planning</p>
                </div>
                <div className={styles.infoCard}>
                  <h5>3. Action Sequencing</h5>
                  <p>High-level goals are decomposed into executable robot action sequences</p>
                </div>
                <div className={styles.infoCard}>
                  <h5>4. Validation</h5>
                  <p>Plans are validated for feasibility before execution</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default CognitivePlanningComponent;
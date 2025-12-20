import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './VLAIntegrationComponent.module.css';

/**
 * VLA Integration Component for complete system visualization
 * Demonstrates the complete Vision-Language-Action pipeline in humanoid robotics
 */
const VLAIntegrationComponent = ({ title = "VLA System Integration Dashboard", description = "Complete visualization of Vision-Language-Action pipeline in humanoid robotics" }) => {
  const [command, setCommand] = useState('');
  const [isProcessing, setIsProcessing] = useState(false);
  const [systemStatus, setSystemStatus] = useState('idle');
  const [visionData, setVisionData] = useState(null);
  const [languageOutput, setLanguageOutput] = useState(null);
  const [actionSequence, setActionSequence] = useState([]);
  const [systemLog, setSystemLog] = useState([]);

  // Simulated vision processing
  const simulateVisionProcessing = (command) => {
    // Simulate object detection results
    const objects = [
      { id: 1, name: 'cup', confidence: 0.95, position: { x: 1.2, y: 0.8, z: 0.0 } },
      { id: 2, name: 'book', confidence: 0.89, position: { x: 0.5, y: 1.5, z: 0.0 } },
      { id: 3, name: 'person', confidence: 0.92, position: { x: -0.5, y: 0.3, z: 0.0 } }
    ];

    const sceneInfo = {
      roomType: 'kitchen',
      navigationPoints: [
        { name: 'entrance', x: 0, y: 0 },
        { name: 'counter', x: 2.0, y: 1.0 },
        { name: 'refrigerator', x: 3.0, y: 0.0 }
      ]
    };

    return { objects, sceneInfo };
  };

  // Simulated language processing with LLM
  const simulateLanguageProcessing = (command, visionData) => {
    // Simulate LLM interpreting the command and generating action plan
    const actionMappings = {
      'kitchen': [
        { id: 1, action: 'NAVIGATE_TO_LOCATION', parameters: { location: 'kitchen_counter' }, description: 'Navigate to kitchen counter' },
        { id: 2, action: 'DETECT_OBJECT', parameters: { object_type: 'cup' }, description: 'Look for cup' },
        { id: 3, action: 'GRASP_OBJECT', parameters: { object_type: 'cup' }, description: 'Pick up the cup' },
        { id: 4, action: 'NAVIGATE_TO_LOCATION', parameters: { location: 'user' }, description: 'Return to user' },
        { id: 5, action: 'PLACE_OBJECT', parameters: { object_type: 'cup', location: 'user' }, description: 'Give cup to user' }
      ],
      'follow': [
        { id: 1, action: 'DETECT_PERSON', parameters: { person_name: 'John' }, description: 'Locate John' },
        { id: 2, action: 'INITIATE_FOLLOWING', parameters: { person_id: 'john_001' }, description: 'Begin following John' },
        { id: 3, action: 'NAVIGATE_TO_LOCATION', parameters: { location: 'office' }, description: 'Follow to office' },
        { id: 4, action: 'STOP_FOLLOWING', parameters: {}, description: 'Stop following' }
      ],
      'entrance': [
        { id: 1, action: 'NAVIGATE_TO_LOCATION', parameters: { location: 'entrance' }, description: 'Go to entrance' },
        { id: 2, action: 'WAIT', parameters: { duration: 60 }, description: 'Wait at entrance' }
      ],
      'book': [
        { id: 1, action: 'NAVIGATE_TO_LOCATION', parameters: { location: 'living_room_table' }, description: 'Go to table' },
        { id: 2, action: 'DETECT_OBJECT', parameters: { object_type: 'book' }, description: 'Find the book' },
        { id: 3, action: 'GRASP_OBJECT', parameters: { object_type: 'book' }, description: 'Pick up book' },
        { id: 4, action: 'NAVIGATE_TO_LOCATION', parameters: { location: 'bookshelf' }, description: 'Go to bookshelf' },
        { id: 5, action: 'PLACE_OBJECT', parameters: { object_type: 'book', location: 'shelf' }, description: 'Place book on shelf' }
      ]
    };

    // Determine which action sequence to use based on command
    let selectedActions = [];
    if (command.toLowerCase().includes('kitchen') || command.toLowerCase().includes('cup')) {
      selectedActions = actionMappings.kitchen;
    } else if (command.toLowerCase().includes('follow') || command.toLowerCase().includes('john')) {
      selectedActions = actionMappings.follow;
    } else if (command.toLowerCase().includes('entrance') || command.toLowerCase().includes('wait')) {
      selectedActions = actionMappings.entrance;
    } else if (command.toLowerCase().includes('book') || command.toLowerCase().includes('shelf')) {
      selectedActions = actionMappings.book;
    } else {
      // Default action sequence
      selectedActions = [
        { id: 1, action: 'PROCESS_COMMAND', parameters: { input: command }, description: `Process command: "${command}"` },
        { id: 2, action: 'SPEAK', parameters: { text: `I'm processing your command: ${command}` }, description: 'Acknowledge command' }
      ];
    }

    return {
      command: command,
      intent: 'Execute requested task',
      action_sequence: selectedActions,
      confidence: 0.85
    };
  };

  // Simulated action execution
  const simulateActionExecution = async (actionSequence) => {
    const results = [];

    for (let i = 0; i < actionSequence.length; i++) {
      const action = actionSequence[i];

      // Update system status
      setSystemStatus(`executing_${action.action.toLowerCase()}`);
      addSystemLog(`Executing: ${action.description}`, 'info');

      // Simulate action taking time
      await new Promise(resolve => setTimeout(resolve, 1500));

      results.push({
        ...action,
        status: 'completed',
        timestamp: new Date().toISOString()
      });

      setActionSequence(prev => {
        const newSequence = [...prev];
        newSequence[i] = { ...newSequence[i], status: 'completed' };
        return newSequence;
      });
    }

    return results;
  };

  const addSystemLog = (message, level = 'info') => {
    const newLogEntry = {
      id: Date.now(),
      timestamp: new Date().toLocaleTimeString(),
      message,
      level
    };
    setSystemLog(prev => [newLogEntry, ...prev.slice(0, 9)]); // Keep last 10 logs
  };

  const handleExecuteCommand = async () => {
    if (!command.trim()) return;

    setIsProcessing(true);
    setSystemStatus('processing');
    addSystemLog(`Received command: "${command}"`, 'info');

    // Step 1: Vision Processing
    setSystemStatus('vision_processing');
    addSystemLog('Starting vision processing...', 'info');
    const visionResults = simulateVisionProcessing(command);
    setVisionData(visionResults);
    addSystemLog('Vision processing complete', 'success');

    // Step 2: Language Processing
    setSystemStatus('language_processing');
    addSystemLog('Starting language processing with LLM...', 'info');
    const languageResults = simulateLanguageProcessing(command, visionResults);
    setLanguageOutput(languageResults);
    setActionSequence(languageResults.action_sequence.map(action => ({ ...action, status: 'pending' })));
    addSystemLog('Language processing complete - action sequence generated', 'success');

    // Step 3: Action Execution
    setSystemStatus('action_execution');
    addSystemLog('Starting action execution...', 'info');
    const actionResults = await simulateActionExecution(languageResults.action_sequence);
    addSystemLog('Action execution complete', 'success');

    setSystemStatus('completed');
    setIsProcessing(false);
  };

  const resetSystem = () => {
    setSystemStatus('idle');
    setCommand('');
    setVisionData(null);
    setLanguageOutput(null);
    setActionSequence([]);
    setIsProcessing(false);
    addSystemLog('System reset', 'info');
  };

  // Sample command suggestions
  const sampleCommands = [
    "Go to the kitchen and bring me the blue cup",
    "Find John and follow him to the office",
    "Navigate to the entrance and wait there",
    "Pick up the book from the table and place it on the shelf",
    "Introduce yourself to the person in the living room"
  ];

  return (
    <div className={clsx('container', styles.integrationContainer)}>
      <div className={clsx('row', styles.integrationRow)}>
        <div className={clsx('col col--12')}>
          <div className={clsx('padding-horiz--md', styles.integrationInterface)}>
            <h3>{title}</h3>
            <p>{description}</p>

            {/* System Status Panel */}
            <div className={styles.systemPanel}>
              <div className={styles.statusDisplay}>
                <h4>System Status</h4>
                <div className={clsx(styles.statusIndicator, {
                  [styles.statusIdle]: systemStatus === 'idle',
                  [styles.statusProcessing]: systemStatus === 'processing',
                  [styles.statusVisionProcessing]: systemStatus.includes('vision'),
                  [styles.statusLanguageProcessing]: systemStatus.includes('language'),
                  [styles.statusActionExecution]: systemStatus.includes('action') || systemStatus.includes('executing'),
                  [styles.statusCompleted]: systemStatus === 'completed'
                })}>
                  <span className={styles.statusText}>
                    {systemStatus === 'idle' && 'Ready to receive commands'}
                    {systemStatus === 'processing' && 'Processing command...'}
                    {systemStatus.includes('vision') && 'Processing visual information...'}
                    {systemStatus.includes('language') && 'Interpreting command with LLM...'}
                    {systemStatus.includes('executing') && 'Executing actions...'}
                    {systemStatus === 'completed' && 'Task completed successfully'}
                  </span>
                </div>
              </div>

              {/* Command Input */}
              <div className={styles.commandInput}>
                <h4>Enter Voice Command</h4>
                <div className={styles.inputGroup}>
                  <input
                    type="text"
                    value={command}
                    onChange={(e) => setCommand(e.target.value)}
                    placeholder="Enter natural language command for the humanoid robot..."
                    className={styles.commandInputField}
                    disabled={isProcessing}
                  />
                  <div className={styles.commandButtons}>
                    <button
                      className={clsx('button button--primary', styles.executeButton)}
                      onClick={handleExecuteCommand}
                      disabled={isProcessing || !command.trim()}
                    >
                      {isProcessing ? 'Processing...' : 'Execute Command'}
                    </button>
                    <button
                      className={clsx('button button--secondary', styles.resetButton)}
                      onClick={resetSystem}
                      disabled={isProcessing}
                    >
                      Reset
                    </button>
                  </div>
                </div>

                {/* Sample Commands */}
                <div className={styles.sampleCommands}>
                  <h5>Try these commands:</h5>
                  <div className={styles.commandChips}>
                    {sampleCommands.map((cmd, index) => (
                      <button
                        key={index}
                        className={styles.commandChip}
                        onClick={() => setCommand(cmd)}
                        disabled={isProcessing}
                      >
                        {cmd}
                      </button>
                    ))}
                  </div>
                </div>
              </div>
            </div>

            {/* Visualization Area */}
            <div className={styles.visualizationArea}>
              <h4>System Visualization</h4>

              {/* Vision Processing Visualization */}
              <div className={styles.pipelineSection}>
                <h5>👁️ Vision Processing</h5>
                {visionData ? (
                  <div className={styles.visionData}>
                    <p><strong>Detecting objects in environment:</strong></p>
                    <div className={styles.objectList}>
                      {visionData.objects.map(obj => (
                        <div key={obj.id} className={styles.objectItem}>
                          <span className={styles.objectName}>{obj.name}</span>
                          <span className={styles.objectConfidence}>Confidence: {(obj.confidence * 100).toFixed(1)}%</span>
                          <span className={styles.objectPosition}>Pos: ({obj.position.x.toFixed(1)}, {obj.position.y.toFixed(1)})</span>
                        </div>
                      ))}
                    </div>
                    <p><strong>Scene:</strong> {visionData.sceneInfo.roomType}</p>
                  </div>
                ) : (
                  <p className={styles.placeholderText}>Waiting for visual input...</p>
                )}
              </div>

              {/* Language Processing Visualization */}
              <div className={styles.pipelineSection}>
                <h5>🧠 Language Processing (LLM)</h5>
                {languageOutput ? (
                  <div className={styles.languageData}>
                    <p><strong>Interpreted Command:</strong> "{languageOutput.command}"</p>
                    <p><strong>Detected Intent:</strong> {languageOutput.intent}</p>
                    <p><strong>Confidence:</strong> {(languageOutput.confidence * 100).toFixed(1)}%</p>
                    <p><strong>Generated Action Sequence:</strong> {languageOutput.action_sequence.length} steps</p>
                  </div>
                ) : (
                  <p className={styles.placeholderText}>Waiting for language interpretation...</p>
                )}
              </div>

              {/* Action Execution Visualization */}
              <div className={styles.pipelineSection}>
                <h5>🤖 Action Execution</h5>
                {actionSequence.length > 0 ? (
                  <div className={styles.actionSequence}>
                    <p><strong>Execution Steps:</strong></p>
                    <div className={styles.actionSteps}>
                      {actionSequence.map((action, index) => (
                        <div
                          key={action.id}
                          className={clsx(styles.actionStep, {
                            [styles.actionCompleted]: action.status === 'completed'
                          })}
                        >
                          <div className={styles.actionHeader}>
                            <span className={styles.actionNumber}>Step {index + 1}</span>
                            <span className={styles.actionType}>{action.action}</span>
                            {action.status === 'completed' && (
                              <span className={styles.completedBadge}>✓ Completed</span>
                            )}
                          </div>
                          <div className={styles.actionDescription}>
                            {action.description}
                          </div>
                          {action.parameters && Object.keys(action.parameters).length > 0 && (
                            <div className={styles.actionParameters}>
                              Parameters: {JSON.stringify(action.parameters)}
                            </div>
                          )}
                        </div>
                      ))}
                    </div>
                  </div>
                ) : (
                  <p className={styles.placeholderText}>Waiting for action sequence...</p>
                )}
              </div>
            </div>

            {/* System Log */}
            <div className={styles.systemLog}>
              <h4>System Log</h4>
              <div className={styles.logEntries}>
                {systemLog.length > 0 ? (
                  systemLog.map(log => (
                    <div key={log.id} className={clsx(styles.logEntry, styles[`log-${log.level}`])}>
                      <span className={styles.logTime}>{log.timestamp}</span>
                      <span className={styles.logMessage}>{log.message}</span>
                    </div>
                  ))
                ) : (
                  <p>No system events yet...</p>
                )}
              </div>
            </div>

            {/* System Information */}
            <div className={styles.systemInfo}>
              <h4>How VLA Systems Work:</h4>
              <div className={styles.infoGrid}>
                <div className={styles.infoCard}>
                  <h5>1. Vision Input</h5>
                  <p>Cameras and sensors capture environmental data, detecting objects, people, and spatial relationships</p>
                </div>
                <div className={styles.infoCard}>
                  <h5>2. Language Processing</h5>
                  <p>LLMs interpret natural language commands and integrate them with visual context to generate action plans</p>
                </div>
                <div className={styles.infoCard}>
                  <h5>3. Action Execution</h5>
                  <p>Generated plans are executed by robot controllers with real-time feedback and adaptation</p>
                </div>
                <div className={styles.infoCard}>
                  <h5>4. Integration Layer</h5>
                  <p>Coordinates between all components, manages state, and ensures smooth operation</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default VLAIntegrationComponent;
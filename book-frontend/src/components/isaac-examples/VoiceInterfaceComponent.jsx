import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './VoiceInterfaceComponent.module.css';

/**
 * Voice Interface Component for VLA module
 * Provides an interactive voice command interface for humanoid robots
 */
const VoiceInterfaceComponent = ({ title = "Voice Command Interface", description = "Speak to control the humanoid robot" }) => {
  const [isListening, setIsListening] = useState(false);
  const [transcript, setTranscript] = useState('');
  const [status, setStatus] = useState('Ready to listen');
  const [commandHistory, setCommandHistory] = useState([]);

  // Simulate voice recognition
  const simulateVoiceRecognition = () => {
    if (!isListening) return;

    // Simulate processing delay
    setTimeout(() => {
      const sampleCommands = [
        "Move forward 2 meters",
        "Turn left and navigate to the kitchen",
        "Pick up the red cup from the table",
        "Follow the person in blue to the office",
        "Stop and wait for further instructions"
      ];

      const randomCommand = sampleCommands[Math.floor(Math.random() * sampleCommands.length)];
      setTranscript(randomCommand);
      setStatus('Command recognized');

      // Add to history
      const newCommand = {
        id: Date.now(),
        command: randomCommand,
        timestamp: new Date().toLocaleTimeString(),
        status: 'recognized'
      };

      setCommandHistory(prev => [newCommand, ...prev.slice(0, 4)]); // Keep last 5 commands

      // Stop listening after command is recognized
      setIsListening(false);
    }, 2000);
  };

  useEffect(() => {
    if (isListening) {
      setStatus('Listening...');
      simulateVoiceRecognition();
    }
  }, [isListening]);

  const handleListenToggle = () => {
    if (isListening) {
      setIsListening(false);
      setStatus('Ready to listen');
    } else {
      setIsListening(true);
      setTranscript('');
      setStatus('Starting to listen...');
    }
  };

  return (
    <div className={clsx('container', styles.voiceInterfaceContainer)}>
      <div className={clsx('row', styles.voiceInterfaceRow)}>
        <div className={clsx('col col--12')}>
          <div className={clsx('padding-horiz--md', styles.voiceInterface)}>
            <h3>{title}</h3>
            <p>{description}</p>

            <div className={styles.voiceControlPanel}>
              <div className={styles.statusDisplay}>
                <div className={clsx(styles.statusIndicator, {
                  [styles.listening]: isListening,
                  [styles.ready]: !isListening && status === 'Ready to listen',
                  [styles.processing]: status.includes('listen') || status.includes('recognize')
                })}>
                  <span className={styles.statusText}>{status}</span>
                </div>
              </div>

              <div className={styles.controls}>
                <button
                  className={clsx('button button--primary', styles.listenButton, {
                    [styles.listeningButton]: isListening
                  })}
                  onClick={handleListenToggle}
                  aria-label={isListening ? "Stop listening" : "Start listening"}
                >
                  {isListening ? (
                    <span className={styles.listeningAnimation}>● Listening...</span>
                  ) : (
                    <span>● Start Voice Command</span>
                  )}
                </button>
              </div>

              {transcript && (
                <div className={styles.transcriptDisplay}>
                  <h4>Recognized Command:</h4>
                  <p className={styles.transcriptText}>{transcript}</p>
                </div>
              )}
            </div>

            {commandHistory.length > 0 && (
              <div className={styles.commandHistory}>
                <h4>Recent Commands:</h4>
                <ul className={styles.historyList}>
                  {commandHistory.map((item) => (
                    <li key={item.id} className={styles.historyItem}>
                      <span className={styles.commandTime}>{item.timestamp}</span>
                      <span className={styles.commandText}>{item.command}</span>
                    </li>
                  ))}
                </ul>
              </div>
            )}

            <div className={styles.voiceTips}>
              <h4>Tips for Voice Commands:</h4>
              <ul>
                <li>Speak clearly and at a moderate pace</li>
                <li>Use simple, direct commands like "move forward" or "turn left"</li>
                <li>Be specific about locations and objects</li>
                <li>Wait for confirmation before issuing another command</li>
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default VoiceInterfaceComponent;
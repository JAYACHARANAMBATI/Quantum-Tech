import React from "react";
import { Link } from "react-router-dom";

const Quantum = () => {
  return (
    <div className="quantum-page">
      <section className="page-hero">
        <div className="page-hero-content">
          <div className="page-badge">Quantum Physics</div>
          <h1 className="page-title">Exploring the Quantum Realm</h1>
          <p className="page-subtitle">
            Discover the fascinating world of quantum mechanics and its revolutionary 
            applications in computing, cryptography, and beyond.
          </p>
        </div>
        <div className="page-hero-image">
          <img 
            src="https://images.unsplash.com/photo-1635070041078-e363dbe005cb?ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D&auto=format&fit=crop&w=2070&q=80" 
            alt="Quantum visualization" 
          />
          <div className="page-hero-overlay"></div>
        </div>
      </section>

      <section className="page-section">
        <div className="container">
          <div className="section-grid">
            <div className="section-content">
              <h2 className="section-title">What is Quantum Physics?</h2>
              <p className="section-text">
                Quantum physics is a fundamental theory in physics that provides a description of the physical properties of nature at the scale of atoms and subatomic particles. It is the foundation of all quantum physics including quantum chemistry, quantum field theory, quantum technology, and quantum information science.
              </p>
              <p className="section-text">
                At the core of quantum mechanics are principles like:
              </p>
              <ul className="feature-list">
                <li>
                  <span className="feature-icon">🔄</span>
                  <div>
                    <h3>Superposition</h3>
                    <p>Quantum particles can exist in multiple states simultaneously until observed.</p>
                  </div>
                </li>
                <li>
                  <span className="feature-icon">🔗</span>
                  <div>
                    <h3>Entanglement</h3>
                    <p>Particles can become correlated in such a way that the quantum state of each particle cannot be described independently.</p>
                  </div>
                </li>
                <li>
                  <span className="feature-icon">📊</span>
                  <div>
                    <h3>Quantum Uncertainty</h3>
                    <p>The Heisenberg Uncertainty Principle states that we cannot simultaneously know both the position and momentum of a particle with perfect precision.</p>
                  </div>
                </li>
              </ul>
            </div>
            <div className="section-image">
              <img 
                src="https://images.unsplash.com/photo-1628595351029-c2bf17511435?ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D&auto=format&fit=crop&w=1632&q=80" 
                alt="Quantum principles" 
              />
            </div>
          </div>
        </div>
      </section>

      <section className="page-section bg-light">
        <div className="container">
          <div className="section-header text-center">
            <h2 className="section-title">Quantum Computing</h2>
            <p className="section-subtitle">
              The Next Frontier in Information Processing
            </p>
          </div>

          <div className="cards-grid">
            <div className="info-card">
              <div className="info-card-icon">⚛️</div>
              <h3>Qubits</h3>
              <p>
                Unlike classical bits that can be either 0 or 1, quantum bits (qubits) can exist in a superposition of both states, enabling exponential computational power.
              </p>
            </div>

            <div className="info-card">
              <div className="info-card-icon">🔐</div>
              <h3>Quantum Cryptography</h3>
              <p>
                Leveraging quantum principles for unbreakable encryption, quantum cryptography provides security based on the fundamental laws of physics.
              </p>
            </div>

            <div className="info-card">
              <div className="info-card-icon">🧠</div>
              <h3>Quantum Algorithms</h3>
              <p>
                Specialized algorithms like Shor's and Grover's can solve certain problems exponentially faster than the best known classical algorithms.
              </p>
            </div>

            <div className="info-card">
              <div className="info-card-icon">🔍</div>
              <h3>Quantum Sensing</h3>
              <p>
                Ultra-precise measurements using quantum systems can detect minute changes in gravity, electromagnetic fields, and more with unprecedented accuracy.
              </p>
            </div>
          </div>
        </div>
      </section>

      <section className="page-section">
        <div className="container">
          <div className="section-header text-center">
            <h2 className="section-title">Our Quantum Research</h2>
            <p className="section-subtitle">
              Pioneering the Future of Quantum Technologies
            </p>
          </div>

          <div className="timeline">
            <div className="timeline-item">
              <div className="timeline-marker"></div>
              <div className="timeline-content">
                <span className="timeline-date">2018</span>
                <h3>Quantum Algorithm Development</h3>
                <p>
                  Our team developed novel quantum algorithms for optimization problems, achieving 40% performance improvement over classical approaches.
                </p>
              </div>
            </div>

            <div className="timeline-item">
              <div className="timeline-marker"></div>
              <div className="timeline-content">
                <span className="timeline-date">2020</span>
                <h3>Quantum Error Correction</h3>
                <p>
                  Breakthrough in quantum error correction techniques, significantly improving qubit stability and coherence time.
                </p>
              </div>
            </div>

            <div className="timeline-item">
              <div className="timeline-marker"></div>
              <div className="timeline-content">
                <span className="timeline-date">2021</span>
                <h3>Quantum-Secure Communications</h3>
                <p>
                  Implemented quantum key distribution network for ultra-secure data transmission across distributed systems.
                </p>
              </div>
            </div>

            <div className="timeline-item">
              <div className="timeline-marker"></div>
              <div className="timeline-content">
                <span className="timeline-date">2023</span>
                <h3>Quantum-Enhanced Machine Learning</h3>
                <p>
                  Integrated quantum computing with neural networks, creating hybrid systems capable of processing complex datasets with higher efficiency.
                </p>
              </div>
            </div>
          </div>
        </div>
      </section>

      <section className="cta-section">
        <div className="container">
          <div className="cta-content text-center">
            <h2>Ready to Explore More Technology Areas?</h2>
            <p>Discover our other cutting-edge technology domains</p>
            <div className="cta-buttons">
              <Link to="/yolo" className="btn btn-primary">
                YOLO Detection
              </Link>
              <Link to="/routing" className="btn btn-primary">
                Routing Systems
              </Link>
              <Link to="/communication" className="btn btn-primary">
                Communication Technologies
              </Link>
            </div>
          </div>
        </div>
      </section>
    </div>
  );
};

export default Quantum; 
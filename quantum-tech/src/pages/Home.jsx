import { Link } from "react-router-dom";

const Home = () => (
  <div className="page-container">
    <section className="hero-section">
      <h1 className="hero-title">Explore the Future of Technology</h1>
      <p className="hero-subtitle">
        Dive into the fascinating world of quantum physics, artificial intelligence, and cutting-edge technology. 
        Discover how these innovations are shaping our future.
      </p>
      <div className="mt-4">
        <Link to="/quantum" className="btn btn-primary me-2">
          <i className="fas fa-atom me-2"></i>
          Explore Quantum Physics
        </Link>
        <Link to="/yolo" className="btn btn-outline-primary">
          <i className="fas fa-camera me-2"></i>
          Discover YOLO Detection
        </Link>
      </div>
    </section>

    <div className="feature-grid">
      <Link to="/quantum" className="feature-card-link">
        <div className="feature-card">
          <i className="fas fa-atom fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
          <h2>Quantum Physics</h2>
          <p>Explore the mysterious world of quantum mechanics, wave-particle duality, and quantum computing applications.</p>
          <div className="mt-3 text-primary">
            <small>Learn more <i className="fas fa-arrow-right"></i></small>
          </div>
        </div>
      </Link>

      <Link to="/yolo" className="feature-card-link">
        <div className="feature-card">
          <i className="fas fa-camera fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
          <h2>YOLO Detection</h2>
          <p>Learn about real-time object detection using YOLO (You Only Look Once) and its applications in computer vision.</p>
          <div className="mt-3 text-primary">
            <small>Learn more <i className="fas fa-arrow-right"></i></small>
          </div>
        </div>
      </Link>

      <Link to="/routing" className="feature-card-link">
        <div className="feature-card">
          <i className="fas fa-route fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
          <h2>Fastest Route Finder</h2>
          <p>Discover advanced algorithms for optimal path finding and their real-world applications in navigation systems.</p>
          <div className="mt-3 text-primary">
            <small>Learn more <i className="fas fa-arrow-right"></i></small>
          </div>
        </div>
      </Link>

      <Link to="/communication" className="feature-card-link">
        <div className="feature-card">
          <i className="fas fa-satellite-dish fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
          <h2>Communication Systems</h2>
          <p>Understand modern communication technologies, from 5G networks to quantum communication protocols.</p>
          <div className="mt-3 text-primary">
            <small>Learn more <i className="fas fa-arrow-right"></i></small>
          </div>
        </div>
      </Link>
    </div>

    <section className="detail-section">
      <h2>About Quantum Tech</h2>
      <p>
        Welcome to Quantum Tech, your premier destination for cutting-edge technology information and education. 
        We are dedicated to making complex technological concepts accessible to everyone, from beginners to experts.
      </p>
      <p>
        Our platform covers a wide range of advanced technologies, providing comprehensive information, tutorials, 
        and insights into the most exciting developments in the tech world.
      </p>
      
      <h2>Our Focus Areas</h2>
      <p>
        At Quantum Tech, we focus on four key technological domains that are revolutionizing our world:
      </p>
      <ul>
        <li>
          <strong>Quantum Physics:</strong> Delve into the fundamental principles of quantum mechanics and discover 
          how this revolutionary field is transforming computing, cryptography, and our understanding of reality.
        </li>
        <li>
          <strong>YOLO Object Detection:</strong> Explore how You Only Look Once (YOLO) algorithms are revolutionizing 
          computer vision, enabling real-time object detection in autonomous vehicles, security systems, and more.
        </li>
        <li>
          <strong>Route Optimization:</strong> Learn about advanced pathfinding algorithms that power navigation 
          systems, logistics operations, and network routing to find the most efficient paths.
        </li>
        <li>
          <strong>Communication Technologies:</strong> Understand the evolution and future of communication systems, 
          from 5G networks to quantum internet, and their impact on global connectivity.
        </li>
      </ul>
      
      <h2>Why Choose Quantum Tech?</h2>
      <p>
        Our platform stands out for its commitment to:
      </p>
      <div className="row">
        <div className="col-md-3">
          <div className="feature-card">
            <i className="fas fa-check-circle fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Accuracy</h3>
            <p>All information is meticulously researched and verified by experts.</p>
          </div>
        </div>
        <div className="col-md-3">
          <div className="feature-card">
            <i className="fas fa-book-reader fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Accessibility</h3>
            <p>Complex concepts are explained in clear, understandable language.</p>
          </div>
        </div>
        <div className="col-md-3">
          <div className="feature-card">
            <i className="fas fa-sync-alt fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Currency</h3>
            <p>We continuously update our content to reflect the latest technological advancements.</p>
          </div>
        </div>
        <div className="col-md-3">
          <div className="feature-card">
            <i className="fas fa-layer-group fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Depth</h3>
            <p>From introductory overviews to detailed technical analyses, we provide content for all knowledge levels.</p>
          </div>
        </div>
      </div>
    </section>

    

    <section className="featured-project-section animate-on-scroll">
      <div className="container">
        <div className="section-header">
          <span className="section-badge">FEATURED PROJECT</span>
          <h2 className="section-title">Quantum-Based Autonomous Navigation System</h2>
          <p className="section-subtitle">
            Explore our cutting-edge autonomous navigation system that combines quantum computing, 
            machine learning and real-time tracking
          </p>
        </div>
        
        <div className="featured-project-card">
          <div className="featured-project-image">
            <img src="https://scitechdaily.com/images/Autonomous-Self-Driving-Car-Technology.jpg" />
          </div>
          <div className="featured-project-content">
            <h3>Project Highlights</h3>
            <ul className="project-highlights">
              <li>
                <i className="fas fa-check-circle"></i>
                <span>Localization using ROS2 and SLAM</span>
              </li>
              <li>
                <i className="fas fa-check-circle"></i>
                <span>Obstacle detection with YOLO models</span>
              </li>
              <li>
                <i className="fas fa-check-circle"></i>
                <span>Path optimization with Dijkstra's Algorithm</span>
              </li>
              <li>
                <i className="fas fa-check-circle"></i>
                <span>Real-time tracking using web APIs</span>
              </li>
              <li>
                <i className="fas fa-check-circle"></i>
                <span>Quantum computing integration for enhanced efficiency</span>
              </li>
            </ul>
            <div className="featured-project-buttons">
              <Link to="/project" className="btn btn-primary pulse-button">
                <i className="fas fa-eye me-2"></i>
                View Project Details
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>

    <section className="contact-section animate-on-scroll">
      <div className="container">
        <div className="section-header">
          <span className="section-badge">CONTACT US</span>
          <h2 className="section-title">Get In Touch</h2>
          <p className="section-subtitle">
            Have questions about our technology solutions? Reach out to us directly.
          </p>
        </div>
        
        <div className="contact-card">
          <div className="contact-info">
            <div className="contact-item">
              <div className="contact-icon">
                <i className="fas fa-user"></i>
              </div>
              <div className="contact-text">
                <h3>Ambati Jaya Charan</h3>
                <p>Lead Technology Consultant</p>
              </div>
            </div>
            
            <div className="contact-item">
              <div className="contact-icon">
                <i className="fas fa-phone"></i>
              </div>
              <div className="contact-text">
                <h3>Phone</h3>
                <p>9640179624</p>
              </div>
            </div>
            
            <div className="contact-item">
              <div className="contact-icon">
                <i className="fas fa-envelope"></i>
              </div>
              <div className="contact-text">
                <h3>Email</h3>
                <p>ambatijayacharan18@gmail.com</p>
              </div>
            </div>
            
            <div className="contact-item">
              <div className="contact-icon">
                <i className="fas fa-map-marker-alt"></i>
              </div>
              <div className="contact-text">
                <h3>Address</h3>
                <p>Vijayawada, Andhra Pradesh, India</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
    
    <section className="cta-section animate-on-scroll">
      <div className="container">
        <h2>Ready to Explore Our Technology Areas?</h2>
        <p>Discover how our cutting-edge solutions can transform your understanding of advanced technologies.</p>
        <div className="cta-buttons">
          <Link to="/quantum" className="btn btn-primary me-3">
            <i className="fas fa-atom me-2"></i>
            Quantum Physics
          </Link>
          <Link to="/yolo" className="btn btn-secondary me-3">
            <i className="fas fa-camera me-2"></i>
            YOLO Detection
          </Link>
          <Link to="/routing" className="btn btn-tertiary me-3">
            <i className="fas fa-route me-2"></i>
            Route Finding
          </Link>
          <Link to="/communication" className="btn btn-quaternary">
            <i className="fas fa-satellite-dish me-2"></i>
            Communication
          </Link>
        </div>
      </div>
    </section>
  </div>
);

export default Home;
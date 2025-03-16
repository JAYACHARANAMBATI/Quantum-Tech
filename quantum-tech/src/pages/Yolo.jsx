import { Link } from "react-router-dom";

const Yolo = () => (
  <div className="page-container">
    <section className="hero-section">
      <h1 className="hero-title">YOLO Object Detection</h1>
      <p className="hero-subtitle">
        Discover the power of real-time object detection using YOLO (You Only Look Once) technology,
        revolutionizing computer vision and AI applications worldwide.
      </p>
    </section>

    <div className="feature-grid">
      <div className="feature-card">
        <i className="fas fa-bolt fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Real-Time Detection</h2>
        <p>YOLO processes images in a single pass, enabling real-time detection at impressive speeds of 45-155 FPS.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-bullseye fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>High Accuracy</h2>
        <p>Latest versions achieve state-of-the-art accuracy while maintaining superior processing speed.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-mobile-alt fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Edge Deployment</h2>
        <p>Optimized for deployment on edge devices, making it ideal for mobile and embedded applications.</p>
      </div>

      <div className="feature-card">
        <i className="fas fa-code-branch fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
        <h2>Open Source</h2>
        <p>YOLO benefits from a large community of developers continuously improving its capabilities.</p>
      </div>
    </div>

    <section className="detail-section">
      <h2>What is YOLO?</h2>
      <p>
        YOLO (You Only Look Once) is a state-of-the-art real-time object detection system that revolutionized 
        the field of computer vision when it was introduced by Joseph Redmon in 2016. Unlike previous 
        object detection systems that applied classifiers to multiple regions within an image, YOLO takes an 
        entirely different approach.
      </p>

      <p>
        The <span className="highlight">key innovation</span> of YOLO is its unified approach: it views object detection as a single regression problem, 
        predicting bounding boxes and class probabilities directly from full images in one evaluation. This approach 
        makes YOLO extremely fast compared to other detection methods.
      </p>

      <div className="row align-items-center my-4">
        <div className="col-md-6">
          <img 
            src="https://miro.medium.com/max/1200/1*ZbmrsQJW-Lp72C5KoTnzUg.jpeg" 
            alt="YOLO detection example showing bounding boxes around detected objects" 
            className="tech-image"
          />
        </div>
        <div className="col-md-6">
          <h3>How YOLO Works</h3>
          <p>
            YOLO divides the input image into a grid and for each grid cell, it predicts:
          </p>
          <ul>
            <li>Bounding box coordinates</li>
            <li>Confidence scores for those boxes</li>
            <li>Class probabilities</li>
          </ul>
          <p>
            This unified architecture allows YOLO to directly optimize detection performance end-to-end.
          </p>
        </div>
      </div>

      <h2>Evolution of YOLO</h2>
      <p>
        Since its introduction, YOLO has gone through several major iterations, each improving upon its predecessor:
      </p>

      <div className="row">
        <div className="col-md-6">
          <h3>YOLOv1-v3</h3>
          <p>
            The original versions developed by Joseph Redmon established YOLO as a groundbreaking approach,
            with YOLOv3 introducing significant improvements with a 53-layer network backbone (Darknet-53).
          </p>
        </div>
        <div className="col-md-6">
          <h3>YOLOv4-v5</h3>
          <p>
            YOLOv4 by Alexey Bochkovskiy incorporated numerous advanced features like Weighted-Residual-Connections 
            and Cross-Stage-Partial-connections. YOLOv5 by Ultralytics further improved performance and usability.
          </p>
        </div>
      </div>

      <div className="row mt-3">
        <div className="col-md-6">
          <h3>YOLOv6-v7</h3>
          <p>
            These versions introduced revolutionary improvements in both speed and accuracy, with YOLOv7 
            establishing new state-of-the-art performance benchmarks on the MS COCO dataset.
          </p>
        </div>
        <div className="col-md-6">
          <h3>Latest Developments</h3>
          <p>
            Recent versions like YOLO-NAS and YOLOv8 have further pushed boundaries with neural architecture 
            search and a more modular codebase, providing even better performance metrics.
          </p>
        </div>
      </div>

      <h2>Applications of YOLO</h2>
      <p>
        YOLO's speed and accuracy make it ideal for numerous real-world applications:
      </p>

      <div className="row">
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-car fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Autonomous Vehicles</h3>
            <p>
              Used for detecting pedestrians, vehicles, traffic signs, and other obstacles in real-time to enable 
              safe navigation.
            </p>
          </div>
        </div>
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-shield-alt fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Surveillance Systems</h3>
            <p>
              Powers security cameras for real-time threat detection, people counting, and unusual activity monitoring.
            </p>
          </div>
        </div>
        <div className="col-md-4">
          <div className="feature-card">
            <i className="fas fa-robot fa-2x mb-3" style={{ color: "var(--primary)" }}></i>
            <h3>Robotics</h3>
            <p>
              Enables robots to identify and interact with objects in their environment for tasks like sorting, picking, and navigation.
            </p>
          </div>
        </div>
      </div>

      <h2>Getting Started with YOLO</h2>
      <p>
        Implementing YOLO in your own projects is easier than you might think:
      </p>

      <div className="row mt-3">
        <div className="col-md-12">
          <div className="card p-4" style={{ background: "rgba(59, 130, 246, 0.05)", borderRadius: "10px" }}>
            <h3>Installation (YOLOv8 example)</h3>
            <pre className="bg-dark text-light p-3 rounded">
              <code>
                pip install ultralytics
              </code>
            </pre>

            <h3 className="mt-4">Basic Usage</h3>
            <pre className="bg-dark text-light p-3 rounded">
              <code>
{`from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n.pt')

# Perform detection
results = model('path/to/image.jpg')

# Process results
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs`}
              </code>
            </pre>
          </div>
        </div>
      </div>

      <div className="text-center mt-5">
        <Link to="/" className="btn btn-outline-primary btn-lg">
          <i className="fas fa-arrow-left me-2"></i>
          Back to Home
        </Link>
      </div>
    </section>
  </div>
);

export default Yolo; 
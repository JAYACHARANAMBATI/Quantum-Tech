import React, { useState, useEffect } from "react";
import { Link, useLocation } from "react-router-dom";

const Navbar = () => {
  const location = useLocation();
  const [isScrolled, setIsScrolled] = useState(false);
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);
  const [activeLink, setActiveLink] = useState("/");

  // Handle scroll effect for navbar
  useEffect(() => {
    const handleScroll = () => {
      if (window.scrollY > 50) {
        setIsScrolled(true);
      } else {
        setIsScrolled(false);
      }
    };

    window.addEventListener("scroll", handleScroll);
    return () => {
      window.removeEventListener("scroll", handleScroll);
    };
  }, []);

  // Update active link based on current location
  useEffect(() => {
    setActiveLink(location.pathname);
  }, [location.pathname]);

  const handleSubmit = (e) => {
    e.preventDefault();
    // Search functionality
    console.log("Search submitted");
  };

  const toggleMobileMenu = () => {
    setIsMobileMenuOpen(!isMobileMenuOpen);
  };

  const closeMobileMenu = () => {
    setIsMobileMenuOpen(false);
  };

  const handleLinkClick = (path) => {
    setActiveLink(path);
  };

  return (
    <nav className={`navbar ${isScrolled ? "navbar-scrolled" : ""}`}>
      <div className="navbar-container">
        <Link to="/" className="navbar-logo" onClick={closeMobileMenu}>
          <span className="navbar-logo-icon">⚛</span>
          <span className="navbar-logo-text">Quantum Tech</span>
        </Link>

        <div className="navbar-mobile-toggle" onClick={toggleMobileMenu}>
          <div className={`hamburger ${isMobileMenuOpen ? "active" : ""}`}>
            <span className="hamburger-line"></span>
            <span className="hamburger-line"></span>
            <span className="hamburger-line"></span>
          </div>
        </div>

        <ul className={`navbar-menu ${isMobileMenuOpen ? "active" : ""}`}>
          <li className="navbar-item">
            <Link 
              to="/" 
              className={`navbar-link ${activeLink === "/" ? "active" : ""}`}
              onClick={closeMobileMenu}
            >
              Home
            </Link>
          </li>
          <li className="navbar-item">
            <Link 
              to="/quantum" 
              className={`navbar-link ${activeLink === "/quantum" ? "active" : ""}`}
              onClick={closeMobileMenu}
            >
              Quantum Physics
            </Link>
          </li>
          <li className="navbar-item">
            <Link 
              to="/yolo" 
              className={`navbar-link ${activeLink === "/yolo" ? "active" : ""}`}
              onClick={closeMobileMenu}
            >
              YOLO Detection
            </Link>
          </li>
          <li className="navbar-item">
            <Link 
              to="/routing" 
              className={`navbar-link ${activeLink === "/routing" ? "active" : ""}`}
              onClick={closeMobileMenu}
            >
              Routing
            </Link>
          </li>
          <li className="navbar-item">
            <Link 
              to="/communication" 
              className={`navbar-link ${activeLink === "/communication" ? "active" : ""}`}
              onClick={() => handleLinkClick("/communication")}
            >
              Communication
            </Link>
          </li>
          <li className="navbar-item">
            <Link 
              to="/project" 
              className={`navbar-link ${activeLink === "/project" ? "active" : ""}`}
              onClick={() => handleLinkClick("/project")}
            >
              Project
            </Link>
          </li>
        </ul>

        <div className={`navbar-search ${isMobileMenuOpen ? "active" : ""}`}>
          <form onSubmit={handleSubmit} className="search-form">
            <input
              type="text"
              placeholder="Search..."
              className="search-input"
            />
            <button type="submit" className="search-button">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="16"
                height="16"
                fill="currentColor"
                viewBox="0 0 16 16"
              >
                <path d="M11.742 10.344a6.5 6.5 0 1 0-1.397 1.398h-.001c.03.04.062.078.098.115l3.85 3.85a1 1 0 0 0 1.415-1.414l-3.85-3.85a1.007 1.007 0 0 0-.115-.1zM12 6.5a5.5 5.5 0 1 1-11 0 5.5 5.5 0 0 1 11 0z" />
              </svg>
            </button>
          </form>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;

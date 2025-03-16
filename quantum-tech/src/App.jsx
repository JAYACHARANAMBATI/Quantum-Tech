import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import Navbar from "./components/Navbar";
import Home from "./pages/Home";
import Quantum from "./pages/Quantum";
import Yolo from "./pages/Yolo";
import Routing from "./pages/Routing";
import Communication from "./pages/Communication";
import Project from "./pages/Project";
import Footer from "./components/Footer";
import ScrollToTop from "./components/ScrollToTop";
import "./App.css";

const App = () => {
  return (
    <Router>
      <ScrollToTop />
      <div className="app">
        <Navbar />
        <main className="main-content">
          <Routes>
            <Route path="/" element={
              <div className="page-container">
                <Home />
              </div>
            } />
            <Route path="/quantum" element={
              <div className="page-container">
                <Quantum />
              </div>
            } />
            <Route path="/yolo" element={
              <div className="page-container">
                <Yolo />
              </div>
            } />
            <Route path="/routing" element={
              <div className="page-container">
                <Routing />
              </div>
            } />
            <Route path="/communication" element={
              <div className="page-container">
                <Communication />
              </div>
            } />
            <Route path="/project" element={
              <div className="page-container">
                <Project />
              </div>
            } />
          </Routes>
        </main>
        <Footer />
      </div>
    </Router>
  );
};

export default App;

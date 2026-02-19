from MainCode.dune_detection import detect_dune

print("Running dune detection test...")

result = detect_dune()

print(f"Distance: {result.distance * 100:.0f} cm")
print(f"Obstacle: {result.obstacle}")
print(f"Dune confidence: {result.dune_confidence:.2f}")
print(f"Frame captured: {result.frame is not None}")

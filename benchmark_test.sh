#!/bin/bash

# Script de test de benchmarking pour Robotik-viewer
# Usage: ./benchmark_test.sh [urdf_file]

echo "=== Test de Benchmarking Robotik-viewer ==="
echo

# Vérifier si un fichier URDF est fourni
if [ $# -eq 0 ]; then
    echo "Usage: $0 <urdf_file>"
    echo "Exemple: $0 data/T12.URDF"
    exit 1
fi

URDF_FILE="$1"

# Vérifier si le fichier existe
if [ ! -f "$URDF_FILE" ]; then
    echo "Erreur: Le fichier URDF '$URDF_FILE' n'existe pas."
    exit 1
fi

echo "Fichier URDF: $URDF_FILE"
echo

# Test 1: Sans profiling (performance normale)
echo "=== Test 1: Performance normale (sans profiling) ==="
echo "Lancement de l'application sans profiling..."
echo "Appuyez sur 'q' pour quitter après quelques secondes"
echo

timeout 10s ./build/Robotik-viewer "$URDF_FILE" 2>/dev/null || true

echo
echo "=== Test 2: Avec profiling activé ==="
echo "Lancement de l'application avec profiling..."
echo "Les statistiques de performance seront affichées toutes les 60 frames"
echo "Appuyez sur 'q' pour quitter après avoir vu les stats"
echo

timeout 15s ./build/Robotik-viewer --profile "$URDF_FILE" 2>/dev/null || true

echo
echo "=== Test 3: Profiling avec FPS élevé ==="
echo "Test avec 120 FPS pour voir l'impact sur les performances..."
echo

timeout 10s ./build/Robotik-viewer --profile --fps 120 "$URDF_FILE" 2>/dev/null || true

echo
echo "=== Test 4: Profiling avec physique à haute fréquence ==="
echo "Test avec physique à 120 Hz..."
echo

timeout 10s ./build/Robotik-viewer --profile --physics 120 "$URDF_FILE" 2>/dev/null || true

echo
echo "=== Tests de benchmarking terminés ==="
echo "Analysez les statistiques affichées pour identifier les goulots d'étranglement."
echo "Les métriques importantes à surveiller:"
echo "- Total Render: Temps total de rendu par frame"
echo "- Robot Render: Temps de rendu du robot"
echo "- Grid Render: Temps de rendu de la grille"
echo "- Physics Update: Temps de mise à jour de la physique"
echo "- Animation Calculation: Temps de calcul de l'animation"
echo "- IK Calculation: Temps de calcul de la cinématique inverse"
echo "- Mesh Loading: Temps de chargement des meshes STL"

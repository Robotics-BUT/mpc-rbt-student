#!/bin/bash
# Tento příkaz zajistí, že pokud cokoliv selže, skript se okamžitě ukončí a nenapíše "úspěšně dokončena"
set -e

echo "Zahajuji kompilaci projektu..."

# 1. Řekneme CMake, ať připraví kompilaci do složky 'build' na základě aktuální složky '.'
cmake -B build -S .

# 2. Spustíme samotnou kompilaci uvnitř složky 'build'
cmake --build build

echo "Kompilace úspěšně dokončena!"

##FreeRTOS na STM32F4DISCOVERY

##Założenia

Projekt został zrealizowany na potrzeby zajęć laboratoryjnych z przedmiotu Podstawy Techniki Mikroprocesorowej (Informatyka na Wydziale Elektrycznym Politechniki Poznańskiej).

Ogólnymi założeniami projektu było poznanie systemu FreeRTOS i oprogramowanie z jego użyciem mikrokontrolera STM32F4DISCOVERY.

##Cele

Podstawowym celem było poznanie składni i działania najważniejszych elementów systemu FreeRTOS takich jak: tworzenie, uruchamianie, opóźnianie, wstrzymywanie, wznawianie, usuwanie oraz zmiana priorytetu zadań. Założeniami dalszymi było stworzenie kilku zadań mających wykonywać operacje na diodach LED.

W fazie finalnej projekt posiada 5 zadań:
* Zapalanie i gaszenie diody,
* Zapalenie i gaszenie diody po opóźnieniu rozpoczęcia działania programu (5 sekund),
* Wykonanie sekwencji zapalania i gaszenia diód (wywoływane losowo, zadanie usuwane po kilku wykonaniach),
* Zapalenie jednej diody po naciśnięciu przycisku i zgaszenie innych (zmiana prorytetów zadań pod wpływem przycisku) oraz wywołanie odpowiedniej sekwencji diód po puszczeniu przycisku,
* Zapalanie i gaszenie diód po osiągnięciu przez akcelerometr odpowiedniej wartości (zawieszenie określonych zadań zadań pod wpływem wartości odczytanej z akcelerometru).

##Realizacja

Projekt napisany przy użyciu języka programowania C oraz środowiska CooCox IDE. Kod był testowany na mikrokontrolerze STM32F4DISCOVERY.

##Wykonanie
* Jakub Podgórski
* Marcin Kwaśnik


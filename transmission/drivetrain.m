%% Script to determine specs of gear wheels
%% writen by Pascal Schoppmann, 20.12.2018, Arosa

close all, clear all, clc

%% Input Parameters

D = 80; %Teilkreisdurchmesser [mm]
b = 6; %Zahnbreite [mm]
L = 14; %Totale Länge [mm]
N = 18; %Durchmesser Nabe [mm]
P = 8; %Durchmesser Bohrung [mm]
H = 3; %Stegbreite [mm]
S = 72.5; %Stegdurchmesser [mm]
density = 1410; % Dichte des materials kg/mm^3

% volume = pi * (D/2)^2 * B + pi * (H/2)^2 * (L-B) - pi * (P/2)^2 * L;
volume = pi * (N^2 - P^2)/4 * L + pi * (S^2 - N^2)/4 * H + pi * (D^2 - S^2)/4 * b;
volume = volume / 1000^3; % convertion to mm^3
weight = volume*density;

disp(['Gewicht des Zahnrades = ', num2str(weight)]);


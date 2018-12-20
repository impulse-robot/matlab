%% Script to determine specs of gear wheels
%% writen by Pascal Schoppmann, 20.12.2018, Arosa

close all, clear all, clc

%% Input Parameters

D = 60; %Teilkreisdurchmesser
B = 6; %Zahnbreite
L = 16; %Totale Länge
H = 44; %Durchmesser Flansch
P = 4; %Durchmesser Bohrung
density = 7700; % Dichte des materials kg/mm^3

volume = pi * (D/2)^2 * B + pi * (H/2)^2 * (L-B) - pi * (P/2)^2 * L;
volume = volume / 1000^3; % convertion to mm^3
weight = volume*density;

disp(['Gewicht des Zahnrades = ', num2str(weight)]);


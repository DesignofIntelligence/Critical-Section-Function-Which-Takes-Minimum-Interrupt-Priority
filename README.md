# Critical-Section-Function-Which-Takes-Minimum-Interrupt-Priority
A critical section function which can take the minimum priority of the interrupt that can preempt the critical section

The goal of this project is to create a function (enter critical section), such that we can define the priority of the interrupt that can preempt the task in the critical section, because in FreeRTOS, the enter critical section function has a constant #defined value in it.

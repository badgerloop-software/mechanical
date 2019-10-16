function x = get_voltage(amps, current_capacity)

x = zeros(21);

 % capacity Percentages from 0% to 100% incremented by 5%
capacity_lut = [3.27 3.61 3.69 3.71 3.73 3.75 3.77 3.79 3.80 3.82 3.84 3.85 3.87 3.91 3.95 3.98 4.02 4.08 4.11 4.15 4.20];
total_capacity = 6; % 6 Ah the starting capacity of the battery
percent_charge = 100 * (current_capacity / total_capacity); % need to figure out the current capacity
percent_charge = 1 + (round(percent_charge / 5) * 5) / 5; % rounding percent change to the nearest multiple of 5 the indexing it to the Capacity array.

x = (capacity_lut(percent_charge) - (0.003 * amps))  * 72 ; % 72 cells at this voltage level 0.003 average internal resistance of each cell.


#ifndef CAPACITY_H
#define	CAPACITY_H

// capacity values
extern long rem_cap; // remaining capacity in uAh
extern long full_cap; // full capacity in mAh
extern int soc; // current calculated SOC

extern long GuessRemainingCap(void);
extern void CountRemainingCapacity(void);
extern void CalcSoc(void);

#endif	/* CAPACITY_H */


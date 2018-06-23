/**
 * @file ds18b20.h
 * @date 10-February-2018
 * @author Alexey Ryabov <6006l1k@gmail.com>
 */

#ifndef __DS18B20_H__
#define __DS18B20_H__

#define DS18B20_NORMALIZE(n) ((((short int)n) * 10) / 16)

int ds18b20_convert_t(const void *);
int ds18b20_read_t(const void *, void *);

#endif /* ~__DS18B20_H__ */

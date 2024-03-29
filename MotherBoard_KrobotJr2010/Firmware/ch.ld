/*
    ChibiOS/RT - Copyright (C) 2010 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/*
 * ST32F103 memory setup.
 */
__main_stack_size__     = 0x0400;
__process_stack_size__  = 0x0400;
__stacks_total_size__   = __main_stack_size__ + __process_stack_size__;

MEMORY
{
    flash : org = 0x08000000, len = 128k
    ram : org = 0x20000000, len = 20k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    .text : ALIGN(16) SUBALIGN(16)
    {
        _text = .;
        KEEP(*(vectors));
        *(.text)
        *(.text.*);
        *(.rodata);
        *(.rodata.*);
        *(.glue_7t);
        *(.glue_7);
        *(.gcc*);
        *(.ctors);
        *(.dtors);
        . = ALIGN(4);
        _etext = .;
    } > flash

    _textdata = _etext;

    .data :
    {
        _data = .;
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        _edata = .;
    } > ram AT > flash

    .bss :
    {
        _bss_start = .;
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram    
    
    /DISCARD/ :
    {
        *(.eh_*)
    }
}

PROVIDE(end = .);
_end            = .;

__heap_base__   = _end;
__heap_end__    = __ram_end__ - __stacks_total_size__;

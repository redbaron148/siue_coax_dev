;**************************************
; 
; All rights reserved.
; 
; Skybotix API is free software: you can redistribute it and/or modify
; it under the terms of the GNU Lesser General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
; 
; Skybotix API is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
; 
; You should have received a copy of the GNU Lesser General Public License
; along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
; 
;*************************************/

.include "p33FJ256GP506.inc"


.section  .text


WriteMem:		; No Argument
	DISI #100
	MOV #0x4001,W0
	MOV W0, NVMCON			
	MOV #0x55,W0		; Write the key sequence
	MOV W0,NVMKEY
	MOV #0xAA,W0
	MOV W0,NVMKEY
	BSET NVMCON,#WR		; Start the write cycle
	NOP
	NOP
1: 	btsc NVMCON, #WR
	bra 1b
	DISI #2
	return 

.global _write_packet
_write_packet:          ; W0 = buffer pointer
		mov [W0], W1
		inc2 W0,W0
		mov [W0], W2
		mov W2, TBLPAG
		inc2 W0,W0

		mov #64, W3
filltable:
		; mov #0x0001,W2
		mov W2, TBLPAG
		tblwtl.b [W0++],[W1++]
		tblwtl.b [W0++],[W1--]
		tblwth.b [W0++],[W1]
		inc2 W1,W1
		dec W3,W3
		bra NZ,filltable

		rcall WriteMem
		return 

.global _erase_memory 
_erase_memory:           ; W0 = buffer pointer
		mov [W0], W1
		inc2 W0,W0
		mov [W0], W2
		mov W2, TBLPAG
		tblwtl W1, [W1]

		disi #100
		mov #0x4042,W0
		mov W0, NVMCON			
		mov #0x55,W0		; Write the key sequence
		mov W0,NVMKEY
		mov #0xAA,W0
		mov W0,NVMKEY
		bset NVMCON,#WR		; Start the write cycle
		nop
		nop
	1: 	btsc NVMCON, #WR
		bra 1b
		disi #2
		return 


.end										; EOF





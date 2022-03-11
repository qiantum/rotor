// Copyright: Qianyan Cai
// License: GPL v3

;({ min, max, abs, floor, ceil, round, PI, cos, sin, sqrt } = Math)
PI2 = Math.PI2 = PI + PI
EPSI = Number.EPSILON = 1 / (1 << 12)

roundepsi = n => round(n / EPSI) * EPSI
Number.prototype.mod = function (n) {
	return ((this % n) + n) % n
}
atan = (x, y) => (Math.atan(y / x) + (x <= 0 ? PI : PI2)) % PI2 // [0,PI2)
dist = (x, y) => Math.sqrt(x * x + y * y)
diff = v => (((v %= PI2) + v) % PI2) - v // [-PI,PI)
diffabs = v => abs(diff(v)) // [0,PI] ((v = abs(v) % PI2) > PI ? PI2 - v : v)

function gcd(a, b) {
	;(a |= 0), (b |= 0)
	for (let c; b; c = a % b, a = b, b = c);
	return a
}
lcm = (a, b) => ((a | 0) * (b | 0)) / gcd(a, b)

Array.prototype.at ??= function (i) {
	return this[(i |= 0) >= 0 ? i : this.length + i]
}
Object.defineProperty(Array.prototype, 'At', {
	enumerable: false,
	get() {
		return i => this[(i |= 0) >= 0 ? i : this.length + i]
	},
})
// [a,b]序列 rev:[b,a]序列
function* sequ(a, b, wrap, oneAll, rev) {
	if (wrap == null) for (; a <= b; rev ? b-- : a++) yield rev ? b : a
	else
		for (a = a.mod(wrap), b = b.mod(wrap); yield rev ? b : a, oneAll || a != b; )
			rev ? (b = (b - 1).mod(wrap)) : (a = (a + 1).mod(wrap)), (oneAll = false)
}
Array.prototype
for (let [k, f] of Object.entries({
	rev() {
		return [...this].reverse()
	},
	*imap(f) {
		let k = 0
		for (let v of this) yield f(v, k++)
	},
	map(f) {
		return [...this.imap(f)]
	},
	*iconcat(s) {
		for (let v of this) yield v
		for (let v of s) yield v
	},
	concat(s) {
		return [...this.iconcat(s)]
	},
	*iclose() {
		let a = this.next().value
		yield a
		for (let v of this) yield v
		yield a
	},
	close(to = this.length, from = 0) {
		return this.values ? ((this[to] = this[from]), this) : [...this.iclose()]
	},
}))
	(Object.getPrototypeOf(sequ).prototype[k] = Object.getPrototypeOf([].keys())[k] = f),
		(Array.prototype[k] ??= f)

Number.prototype.bfind = function (s, prop, epsi) {
	let l = 0,
		h = s.length - 1
	for (let m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = this - (prop != null ? s[m][prop] : s[m]))
		if (epsi ? abs(c) <= EPSI : c == 0) return m
		c <= 0 ? (h = m - 1) : (l = m + 1)
	}
	let m = (prop != null ? s[h]?.[prop] + s[l]?.[prop] : s[h] + s[l]) / 2 // 超出为NaN
	return this < m ? h + 0.25 : l - 0.25
}

function cross(ax, ay, bx, by, cx, cy, dx, dy, co) {
	let abc = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
	let abd = (ax - dx) * (by - dy) - (ay - dy) * (bx - dx)
	let cda = (cx - ax) * (dy - ay) - (cy - ay) * (dx - ax)
	let cdb = abc - abd + cda
	if (abc * abd >= 0 || cda * cdb >= 0) {
		if (!co || (abc && abd)) return [null, null, abc, abd, cda, cdb]
		return cda == 0 ? [ax, ay, abc, abd, cda, cdb] : [bx, by, abc, abd, cda, cdb]
	}
	let t = cda / (abd - abc)
	return [ax + t * (bx - ax), ay + t * (by - ay), abc, abd, cda, cdb]
}
function area(s) {
	let [x0, y0] = (s = s.values?.() ?? s).next().value
	let [xx, yy] = [x0, y0]
	let a = 0
	for (let [x, y] of s) (a += (xx - x) * (yy + y)), (xx = x), (yy = y)
	a = (a + (xx - x0) * (yy + y0)) / 2 // 闭合
	return roundepsi(a)
}

function fillhole(hole) {
	let ii = this.findIndex(v => v != hole)
	if (ii < 0) throw 'all hole'
	let len = this.length
	let m = 0
	for (let j = ii, i = ii, k, n; (i = (i + 1) % len) != ii; j = i)
		if (this[i] == hole) {
			for (k = i; this[(k = (k + 1) % len)] == hole; );
			for (n = (k - j + len) % len, l = 1; l < n; l++, i = (i + 1) % len)
				this[i] = this[j] + ((this[k] - this[j]) * l) / n
			m += n - 1
		}
	return m
}
function matran(row) {
	let col = []
	let i = 0
	for (let r of row) {
		let j = 0
		for (let c of r) (col[j++] ??= [])[i] = c
		i++
	}
	return col
}

// TODO 非闭合曲线不适用，需改进
if (false) {
	// 正向曲线集求内包络线 curves:[[ [A, R, X, Y] ]] tt:正向步进、可卷
	function enveCurve(curves, wrap, tt = Tick_.keys()) {
		let M
		for (let cc of curves) {
			cc.values ?? (cc = [...cc])
			if (!M) {
				M = cc // 初始化包络
				for (let moved, m = 1; m < M.length; m++)
					if (M[m - 1][0] > M[m - 1][0])
						if (moved || M[m - 1][0] < PI / 2 || M[m][0] > PI / 2) throw 'err curve A'
						else M.push(M.splice(0, m)), (moved = m = 1)
				continue
			}
			cc.forEach(c => (c[4] = c[0].bfind(M, 0)))
			let C = []
			for (let c of cc.keys()) {
				let [a, r, x, y, I0] = cc[c++]
				if (!cc[c] && !wrap) continue
				let [a_, r_, x_, y_, I9] = cc[c] ?? cc[0]
				;(I0 = floor(I0)), (I9 = ceil(I9)) // 遍历曲线连续段，对应包络连续段
				let IC = I0 + 1
				if (I0 < 0) I0 = wrap ? M.length - 1 : 0
				if (I9 == M.length) I9 = wrap ? 0 : M.length - 1
				for (let I = I0, I1, I_; I != I9; I = I_) {
					let [A, R, X, Y] = M[I]
					let [A_, R_, X_, Y_] = M[(I_ = I1 = I + 1)] ?? M[(I_ = 0)]
					// 求交点和半径
					let [SX, SY, ABc, ABd, cdA, cdB] = cross(X, Y, X_, Y_, x, y, x_, y_)
					if (I_ != I9 && cdB < -EPSI && I_) C.push([0 / 0, 0, 0, 0, I_]) // 去掉长半径包络点
					if (I == I0 && ABc > EPSI) C.push([a, r, x, y, IC]) // 短半径曲线点
					if (I_ == I9 && ABd > EPSI) C.push([a_, r_, x_, y_, I1]) // 短半径曲线点
					if (SX != null) C.push([atan(SX, SY), dist(SX, SY), SX, SY, I1]) //交点
				}
			}
			C.sort((c, d) => c[4] - d[4] || c[0] - d[0]) // 下标和角度顺序
			for (let c = 1; c < C.length; c++) if (M[m][0] < M[m - 1][0]) throw 'err curve A'
			let i = 0 // 加减点
			for (let c of C)
				if (c[0] != c[0]) M.splice(c[4] + i--, 1)
				else M.splice(c[4] + i++, 0, c), c.length--
			M.push((([A, ...M]) => [A + PI2, ...M])(M[0])) // 闭合
			i = 0 // 合并重合点
			for (let m of M)
				if (m[0] < M[i][0]) throw 'err a'
				else if (m[0] - M[i][0] > EPSI && abs(m[1] - M[i][1]) > EPSI) M[++i] = m
			M.length = ++i
		}
		tt.values ?? (tt = [...tt])
		function* XY(T, ttt = tt, x = GX(T), y = GY(T)) {
			ttt = ttt.values?.() ?? ttt
			let t = ttt.next().value
			if (t == null) return
			let m = round(Tick_[t].bfind(M, 0))
			for (let t_ of ttt)
				do
					for (let A, T_ = Tick_[t_] + (t > t_ && PI2); (A = M[m]?.[0]) <= T_; m++)
						yield [x + M[m][1] * cos(T + A), y + M[m][1] * sin(T + A)]
				while (t > (t = t_) && !(m = 0))
		}
		return (XY.count = M.length - 1), XY
	}
}

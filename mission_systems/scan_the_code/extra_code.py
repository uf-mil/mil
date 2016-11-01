    def line_intersection(self, p1, vec1, p2, vec2):
        x1, y1 = p1
        x2, y2 = vec1[0] * 5 + x1, vec1[1] * 5 + y1
        x11, y11 = p2
        x22, y22 = vec2[0] * 5 + x11, vec2[1] * 5 + y11
        m1 = (y2 - y1) / (x2 - x1 + 1E-100)
        m2 = (y22 - y11) / (x22 - x11 + 1E-100)
        if abs(m1) < 1E-50:
            return x22, y2
        if abs(m2) < 1E-50:
            return x2, y22

        x = (y1 - y11 + m2 * x11 - m1 * x1) / (m2 - m1)
        y = m1 * (x - x1) + y1
        return x, y
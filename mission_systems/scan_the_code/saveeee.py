def get_vertical_lines(self, edges, img):
        # $$$$$$$$$$$$$$$$$$$$ DEBUG - REMOVE IMG PARAM $$$$$$$$$$$$$$$$$$$$$$
        e = img.copy()
        img1 = img.copy()
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
        lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=30, minLineLength=10)
        if lines is None:
            print "No lines at all"
            return False, None

        vert_lines = []
        for x1, y1, x2, y2 in lines[0]:
            slope = (y2 - y1) / (x2 - x1 + 1E-100)
            if abs(slope) > .8:
                xoi, yoi = x1, y1
                top_x, top_y = 0, 0
                bottom_x, bottom_y = 0, 0
                count = 0
                up = True
                # Cleaning up the line estimate
                while self.in_range(edges, xoi, yoi) and count < 30:
                    print xoi, yoi
                    if not self.in_range(edges, xoi - 5, yoi - 1) or not self.in_range(edges, xoi + 5, yoi - 1):
                        if up:
                            print "switch"
                            up = False
                            top_x = xoi
                            top_y = yoi
                            xoi, yoi = x1, y1
                            continue
                        else:
                            bottom_x = xoi
                            bottom_y = yoi
                            break

                    r = 3
                    if up:
                        check = edges[yoi - 1: yoi, xoi - r: xoi + r].flatten()
                    else:
                        check = edges[yoi: yoi + 1, xoi - r: xoi + r].flatten()
                    check = map(lambda x: abs(x), check)
                    index, value = max(enumerate(check), key=operator.itemgetter(1))
                    if value < 255:
                        print "switch"
                        if up:
                            up = False
                            top_x = xoi
                            top_y = yoi
                            xoi, yoi = x1, y1
                        else:
                            bottom_x = xoi
                            bottom_y = yoi
                            break
                    r = 3
                    xoi += - r + index
                    if up:
                        yoi += - r
                    else:
                        yoi -= -r
                    count += 1
                    # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                    # cv2.circle(img1, (xoi, yoi), 1, (255, 0, 0), -1)
                    # self.debug.add_image(img1, "sup", wait=33)
                    # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

                vert_lines.append((top_y, bottom_y - 5, bottom_y - top_y, bottom_x - top_x))
                # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            cv2.line(e, (x1, y1), (x2, y2), (0, 0, 255), 2)
        self.debug.add_image(e, "linees")
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
        if len(vert_lines) < 4:
            return False, None
        return True, vert_lines

    def get_loi(self, starting_height, vec, w):
        line_count = 0
        loi_x = []
        for i in range(0, w, 1):
            if self.edges[starting_height, i] > 0:
                # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                # cv2.circle(self.edges_1, (i, starting_height), 1, (255, 0, 0), -1)
                # self.debug.add_image(self.edges_1, "horiz", wait=33)
                # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

                # Check and see if there is a line there
                hits = 0
                for j in range(-6, 6, 3):
                    a, b = map(lambda vec: self.toint(vec * j), vec)
                    # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                    # cv2.circle(self.edges_2, (a + i, b + starting_height), 1, (255, 0, 0), -1)
                    # self.debug.add_image(self.edges_2, "horiz_points")
                    # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
                    if self.in_range(self.edges, a + i - 1, b + starting_height) and self.in_range(self.edges,
                                                                                                   a + i + 1, b + starting_height):
                        buff = self.edges[b + starting_height, a + i - 1:a + i + 1]
                    else:
                        continue
                    if sum(buff) > 0:
                        hits += 1
                        # print "Yay! you got a line! Hits = {}".format(hits)
                if hits >= 2:
                    line_count += 1
                    loi_x.append(i)
        return line_count, loi_x
